#include "rlm3-gps.h"
#include "rlm3-task.h"
#include "rlm3-uart.h"
#include "rlm3-gpio.h"
#include "logger.h"
#include "Assert.h"


#ifndef RLM3_GPS_BUFFER_SIZE
#define RLM3_GPS_BUFFER_SIZE 1024
#endif


LOGGER_ZONE(GPS);


typedef enum State
{
	STATE_IDLE,
	STATE_LOST,
	STATE_LOST_A,
	STATE_LOST_B,
	STATE_IGNORE,

	STATE_PREFIX_0,
	STATE_PREFIX_1,
	STATE_LENGTH_0,
	STATE_LENGTH_1,
	STATE_TYPE,
	STATE_BODY,
	STATE_CHECKSUM,
	STATE_SUFFIX_0,
	STATE_SUFFIX_1,
} State;


static const uint8_t PREFIX_0 = 0xA0;
static const uint8_t PREFIX_1 = 0xA1;
static const uint8_t SUFFIX_0 = 0x0D;
static const uint8_t SUFFIX_1 = 0x0A;


static volatile uint8_t g_buffer[RLM3_GPS_BUFFER_SIZE] __attribute((aligned(4)));
static volatile RLM3_Task g_client_task = NULL;

static volatile size_t g_server_cursor = 0;
static volatile size_t g_server_end = 0;
static volatile size_t g_server_next = 0;
static volatile size_t g_client_cursor = 0;
static volatile size_t g_client_next = 0;

static volatile State g_tx_state = STATE_IDLE;
static volatile const RLM3_GPS_MESSAGE* g_tx_message = NULL;
static uint8_t g_tx_checksum = 0;
static size_t g_tx_offset = 0;

static volatile State g_rx_state = STATE_IDLE;
static volatile RLM3_GPS_MESSAGE* g_rx_message = NULL;
static size_t g_rx_payload_size = 0;
static size_t g_rx_offset = 0;
static uint8_t g_rx_checksum = 0;


extern void RLM3_GPS_Init()
{
	if (RLM3_UART2_IsInit())
		RLM3_UART2_Deinit();

	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	HAL_GPIO_WritePin(GPS_RESET_GPIO_Port, GPS_RESET_Pin, GPIO_PIN_RESET);

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin = GPS_PULSE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPS_PULSE_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPS_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPS_RESET_GPIO_Port, &GPIO_InitStruct);

	g_client_task = NULL;

	g_server_cursor = 0;
	g_server_end = 0;
	g_server_next = 0;
	g_client_cursor = 0;
	g_client_next = 0;

	g_tx_state = STATE_IDLE;
	g_tx_message = NULL;

	g_rx_state = STATE_IDLE;
	g_rx_message = NULL;

	// Reset the GPS module.
	RLM3_Delay(10);
	HAL_GPIO_WritePin(GPS_RESET_GPIO_Port, GPS_RESET_Pin, GPIO_PIN_SET);
	RLM3_Delay(1000);

	// Start listening to the GPS module.
	RLM3_UART2_Init(115200);
}

extern void RLM3_GPS_Deinit()
{
	HAL_GPIO_DeInit(GPS_PULSE_GPIO_Port, GPS_PULSE_Pin);
	HAL_GPIO_DeInit(GPS_RESET_GPIO_Port, GPS_RESET_Pin);

	RLM3_UART2_Deinit();
}

extern bool RLM3_GPS_IsInit()
{
	return RLM3_UART2_IsInit();
}

extern const RLM3_GPS_MESSAGE* RLM3_GPS_GetNextMessage(size_t timeout_ms)
{
	// Release any previous message.
	g_client_cursor = g_client_next;
	g_client_task = RLM3_GetCurrentTask();

	// Wait for at least one message to be available.
	RLM3_Time start_time = RLM3_GetCurrentTime();
	while (g_server_cursor == g_client_cursor)
	{
		if (!RLM3_TakeUntil(start_time, timeout_ms))
		{
			g_client_task = NULL;
			return NULL;
		}
	}
	g_client_task = NULL;

	// Wrap to the beginning of the buffer if we are at the end of the server data.
	if (g_client_cursor > g_server_cursor && g_client_cursor == g_server_end)
		g_client_cursor = 0;

	// Get the message set up.
	RLM3_GPS_MESSAGE* message = (RLM3_GPS_MESSAGE*)(g_buffer + g_client_cursor);
	size_t message_size = message->payload_length + 2;
	size_t block_size = (message_size + 3) / 4 * 4;
	g_client_next = g_client_cursor + block_size;
	return message;
}

extern bool RLM3_GPS_SendMessage(const RLM3_GPS_MESSAGE* message)
{
	if (message == NULL)
		return false;
	if (message->payload_length == 0)
		return false;
	if (g_tx_state != STATE_IDLE)
		return false;
	if (g_client_task != NULL)
		return false;

	g_client_task = RLM3_GetCurrentTask();
	g_tx_message = message;

	g_tx_state = STATE_PREFIX_0;
	RLM3_UART2_EnsureTransmit();

	while (g_tx_state != STATE_IDLE)
		RLM3_Take();

	ASSERT(g_tx_message == NULL);
	g_client_task = NULL;
	return true;
}

static RLM3_GPS_MESSAGE* AllocateRxMessage(size_t payload_length)
{
	size_t message_size = payload_length + 2;
	size_t block_size = (message_size + 3) / 4 * 4; // Make sure each message is aligned to a 4 byte boundary.

// TODO: This is the only place the server will modify the client cursor.  Verify this is absolutely safe before enabling it.
//	if (g_client_cursor == g_server_cursor)
//	{
//		// The buffer is empty, so reset all writes to the beginning.  This avoids wasting space at the end of the buffer when we wrap large messages.
//		g_client_cursor = 0;
//		g_server_cursor = 0;
//	}

	if (g_client_cursor <= g_server_cursor)
	{
		if (g_server_cursor + block_size <= RLM3_GPS_BUFFER_SIZE)
		{
			// There is room at the end of the buffer.
			g_server_next = g_server_cursor + block_size;
			return (RLM3_GPS_MESSAGE*)(g_buffer + g_server_cursor);
		}
		if (block_size < g_client_cursor)
		{
			// There is room if we wrap the buffer.
			g_server_end = g_server_cursor;
			g_server_next = block_size;
			return (RLM3_GPS_MESSAGE*)(g_buffer);
		}
	}
	else
	{
		if (g_client_cursor - g_server_cursor > message_size)
		{
			// There is room in the middle of the buffer.
			g_server_next = g_server_cursor + block_size;
			return (RLM3_GPS_MESSAGE*)(g_buffer + g_server_cursor);
		}
	}

	return NULL;
}

static void DebugPrintHex(uint8_t data)
{
	static const char* g_hex_chars = "0123456789ABCDEF";
	RLM3_DebugOutputFromISR(g_hex_chars[(data >> 4) & 0x0F]);
	RLM3_DebugOutputFromISR(g_hex_chars[(data >> 0) & 0x0F]);
	RLM3_DebugOutputFromISR(' ');
}

extern void RLM3_UART2_ReceiveCallback(uint8_t data)
{
	if (IS_LOG_TRACE())
		DebugPrintHex(data);

	State next_state = STATE_LOST;
	switch (g_rx_state)
	{
	case STATE_IDLE:
		if (data == PREFIX_0)
			next_state = STATE_PREFIX_1;
		break;

	case STATE_LOST:
	case STATE_LOST_A:
		if (data == PREFIX_0)
			next_state = STATE_LOST_B;
		else
			next_state = STATE_LOST_A;
		break;

	case STATE_LOST_B:
		if (data == PREFIX_0)
			next_state = STATE_LOST_B;
		else if (data == PREFIX_1)
			next_state = STATE_LENGTH_0;
		else
			next_state = STATE_LOST_A;
		break;

	case STATE_PREFIX_1:
		if (data == PREFIX_1)
			next_state = STATE_LENGTH_0;
		break;

	case STATE_LENGTH_0:
		g_rx_payload_size = data << 8;
		next_state = STATE_LENGTH_1;
		break;

	case STATE_LENGTH_1:
		g_rx_payload_size += data;
		g_rx_message = AllocateRxMessage(g_rx_payload_size);
		if (g_rx_message == NULL)
			RLM3_GPS_ErrorCallback(RLM3_GPS_ERROR_BUFFER_FULL);
		if (g_rx_message != NULL)
			g_rx_message->payload_length = g_rx_payload_size;
		if (g_rx_payload_size != 0)
			next_state = STATE_TYPE;
		break;

	case STATE_TYPE:
		if (g_rx_message != NULL)
			g_rx_message->message_type = data;
		g_rx_offset = 0;
		g_rx_checksum = data;
		next_state = STATE_BODY;
		if (g_rx_payload_size <= 1)
			next_state = STATE_CHECKSUM;
		break;

	case STATE_BODY:
		if (g_rx_message != NULL)
			g_rx_message->data[g_rx_offset] = data;
		g_rx_offset++;
		g_rx_checksum ^= data;
		next_state = STATE_BODY;
		if (g_rx_offset + 1 >= g_rx_payload_size)
			next_state = STATE_CHECKSUM;
		break;

	case STATE_CHECKSUM:
		if (g_rx_checksum != data)
		{
			LOG_WARN("Checksum Error %x vs %x", (int)g_rx_checksum, (int)data);
			RLM3_GPS_ErrorCallback(RLM3_GPS_ERROR_CHECKSUM_FAIL);
			g_rx_message = NULL;
		}
		next_state = STATE_SUFFIX_0;
		break;

	case STATE_SUFFIX_0:
		if (data == SUFFIX_0)
			next_state = STATE_SUFFIX_1;
		break;

	case STATE_SUFFIX_1:
		if (data == SUFFIX_1)
		{
			if (g_rx_message != NULL)
			{
				// The complete message was received.  Pass it along to the application.
				g_server_cursor = g_server_next;
				RLM3_GiveFromISR(g_client_task);
			}
			next_state = STATE_IDLE;
		}
		break;

	default:
		LOG_WARN("RX STATE %d", g_rx_state);
		RLM3_GPS_ErrorCallback(RLM3_GPS_ERROR_INTERNAL);
		next_state = STATE_LOST_A;
	}

	if (next_state == STATE_LOST)
	{
		LOG_WARN("RX LOST %d %x", g_rx_state, data);
		RLM3_GPS_ErrorCallback(RLM3_GPS_ERROR_PROTOCOL_FAIL);
	}
	g_rx_state = next_state;
}

extern bool RLM3_UART2_TransmitCallback(uint8_t* data_to_send)
{
	switch (g_tx_state)
	{
	case STATE_IDLE:
		return false;

	case STATE_PREFIX_0:
		*data_to_send = PREFIX_0;
		g_tx_state = STATE_PREFIX_1;
		return true;

	case STATE_PREFIX_1:
		*data_to_send = PREFIX_1;
		g_tx_state = STATE_LENGTH_0;
		return true;

	case STATE_LENGTH_0:
		ASSERT(g_tx_message != NULL);
		*data_to_send = (g_tx_message->payload_length >> 8) & 0xFF;
		g_tx_state = STATE_LENGTH_1;
		return true;

	case STATE_LENGTH_1:
		ASSERT(g_tx_message != NULL);
		*data_to_send = (g_tx_message->payload_length >> 0) & 0xFF;
		g_tx_state = STATE_TYPE;
		return true;

	case STATE_TYPE:
		ASSERT(g_tx_message != NULL);
		*data_to_send = g_tx_message->message_type;
		g_tx_checksum = g_tx_message->message_type;
		g_tx_offset = 0;
		g_tx_state = STATE_BODY;
		if (g_tx_message->payload_length <= 1)
			g_tx_state = STATE_CHECKSUM;
		return true;

	case STATE_BODY:
		ASSERT(g_tx_message != NULL);
		*data_to_send = g_tx_message->data[g_tx_offset];
		g_tx_checksum ^= g_tx_message->data[g_tx_offset];
		g_tx_offset++;
		if (g_tx_offset + 1 >= g_tx_message->payload_length)
			g_tx_state = STATE_CHECKSUM;
		return true;

	case STATE_CHECKSUM:
		*data_to_send = g_tx_checksum;
		g_tx_state = STATE_SUFFIX_0;
		return true;

	case STATE_SUFFIX_0:
		*data_to_send = SUFFIX_0;
		g_tx_state = STATE_SUFFIX_1;
		return true;

	case STATE_SUFFIX_1:
		*data_to_send = SUFFIX_1;
		g_tx_message = NULL;
		g_tx_state = STATE_IDLE;
		RLM3_GiveFromISR(g_client_task);
		return true;

	default:
		LOG_WARN("TX STATE %d", g_tx_state);
		RLM3_GPS_ErrorCallback(RLM3_GPS_ERROR_INTERNAL);
		g_tx_state = STATE_IDLE;
		return false;
	}
}

extern void RLM3_UART2_ErrorCallback(uint32_t status_flags)
{
	LOG_WARN("UART ERROR %x", (unsigned int)status_flags);
	g_rx_state = STATE_LOST;
	RLM3_GPS_ErrorCallback(RLM3_GPS_ERROR_CHANNEL_FAIL);
}

extern void RLM3_EXTI12_Callback()
{
	RLM3_GPS_PulseCallback();
}

extern __attribute((weak)) void RLM3_GPS_PulseCallback()
{
}

extern __attribute((weak)) void RLM3_GPS_ErrorCallback(RLM3_GPS_Error error)
{
}

