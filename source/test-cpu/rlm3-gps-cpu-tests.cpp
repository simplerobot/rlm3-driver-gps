#include "Test.hpp"
#include "rlm3-gps.h"
#include "rlm3-gpio.h"
#include "rlm3-uart.h"
#include "rlm3-task.h"
#include "rlm3-sim.hpp"
#include "rlm3-bytes.h"
#include <vector>


static size_t g_pulse_count = 0;
static std::vector<RLM3_GPS_Error> g_errors;

extern void RLM3_GPS_PulseCallback()
{
	g_pulse_count++;
	SIM_Give();
}

extern void RLM3_GPS_ErrorCallback(RLM3_GPS_Error error)
{
	g_errors.push_back(error);
	SIM_Give();
}

TEST_CASE(RLM3_GPS_Init_HappyCase)
{
	ASSERT(!RLM3_GPS_IsInit());
	RLM3_GPS_Init();

	ASSERT(RLM3_GPS_IsInit());
	ASSERT(SIM_RLM3_UART2_GetBaudrate() == 115200);

	ASSERT(SIM_GPIO_IsClockEnabled(GPIOB));
	ASSERT(SIM_GPIO_IsClockEnabled(GPIOG));

	ASSERT(SIM_GPIO_GetMode(GPIOB, GPIO_PIN_12) == GPIO_MODE_OUTPUT_PP);
	ASSERT(SIM_GPIO_GetPull(GPIOB, GPIO_PIN_12) == GPIO_NOPULL);

	ASSERT(SIM_GPIO_GetMode(GPIOG, GPIO_PIN_12) == GPIO_MODE_IT_RISING);
	ASSERT(SIM_GPIO_GetPull(GPIOG, GPIO_PIN_12) == GPIO_PULLUP);

	ASSERT(RLM3_GetCurrentTime() >= 1000);
}

TEST_CASE(RLM3_GPS_DeInit_HappyCase)
{
	RLM3_GPS_Init();

	ASSERT(RLM3_GPS_IsInit());
	RLM3_GPS_Deinit();

	ASSERT(!RLM3_GPS_IsInit());
	ASSERT(!RLM3_UART2_IsInit());

	ASSERT(SIM_RLM3_UART2_GetBaudrate() == 115200);

	ASSERT(SIM_GPIO_IsClockEnabled(GPIOB));
	ASSERT(SIM_GPIO_IsClockEnabled(GPIOG));

	ASSERT(SIM_GPIO_GetMode(GPIOB, GPIO_PIN_12) == GPIO_MODE_DISABLED);
	ASSERT(SIM_GPIO_GetMode(GPIOG, GPIO_PIN_12) == GPIO_MODE_DISABLED);
}

TEST_CASE(RLM3_GPS_Pulse_HappyCase)
{
	g_pulse_count = 0;
	SIM_GPIO_Interrupt(GPIOG, GPIO_PIN_12);
	RLM3_GPS_Init();

	ASSERT(g_pulse_count == 0);
	RLM3_Take();
	ASSERT(g_pulse_count == 1);
}

TEST_CASE(RLM3_GPS_GetNextMessage_HappyCase)
{
	uint8_t raw_msg_02[] = { 0xA0, 0xA1, 0x00, 0x02, 0x02, 0x00, 0x02, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_msg_02, sizeof(raw_msg_02));
	RLM3_GPS_Init();

	const RLM3_GPS_MESSAGE* msg = RLM3_GPS_GetNextMessage(10);

	ASSERT(msg != nullptr);
	ASSERT(msg->payload_length == 2);
	ASSERT(msg->message_type == RLM3_GPS_MESSAGE_TYPE_02_QUERY_SOFTWARE_VERSION);

	auto msg_02 = (const RLM3_GPS_MESSAGE_02_QUERY_SOFTWARE_VERSION*)msg;
	ASSERT(msg_02->software_type == 0);
}

TEST_CASE(RLM3_GPS_GetNextMessage_NoMessages)
{
	RLM3_GPS_Init();

	const RLM3_GPS_MESSAGE* msg = RLM3_GPS_GetNextMessage(10);

	ASSERT(msg == nullptr);
}

TEST_CASE(RLM3_GPS_SendMessage_HappyCase)
{
	uint8_t raw_msg_02[] = { 0xA0, 0xA1, 0x00, 0x02, 0x02, 0x01, 0x03, 0x0D, 0x0A };
	SIM_RLM3_UART2_TransmitRaw(raw_msg_02, sizeof(raw_msg_02));
	RLM3_GPS_MESSAGE_02_QUERY_SOFTWARE_VERSION msg;
	RLM3_GPS_SET_MESSAGE_PAYLOAD_SIZE(msg);
	msg.message_type = RLM3_GPS_MESSAGE_TYPE_02_QUERY_SOFTWARE_VERSION;
	msg.software_type = 1;
	RLM3_GPS_Init();

	ASSERT(RLM3_GPS_SendMessage((RLM3_GPS_MESSAGE*)&msg));
}

TEST_CASE(RLM3_GPS_SendMessage_Null)
{
	RLM3_GPS_Init();

	ASSERT(!RLM3_GPS_SendMessage(nullptr));
}

TEST_CASE(RLM3_GPS_Error_ChecksumFail)
{
	g_errors.clear();
	uint8_t raw_msg_02[] = { 0xA0, 0xA1, 0x00, 0x02, 0x02, 0x00, 0x03, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_msg_02, sizeof(raw_msg_02));
	RLM3_GPS_Init();

	const RLM3_GPS_MESSAGE* msg = RLM3_GPS_GetNextMessage(10);

	ASSERT(g_errors.size() == 1);
	ASSERT(g_errors[0] == RLM3_GPS_ERROR_CHECKSUM_FAIL);
	ASSERT(msg == nullptr);
}

TEST_CASE(RLM3_GPS_Error_ChannelFail)
{
	g_errors.clear();
	SIM_RLM3_UART2_Error(0);
	RLM3_GPS_Init();

	const RLM3_GPS_MESSAGE* msg = RLM3_GPS_GetNextMessage(10);

	ASSERT(g_errors.size() == 1);
	ASSERT(g_errors[0] == RLM3_GPS_ERROR_CHANNEL_FAIL);
	ASSERT(msg == nullptr);
}

TEST_CASE(RLM3_GPS_Error_ProtocolFail_A)
{
	g_errors.clear();
	uint8_t invalid_msg[] = { 0xA1, 0xA1, 0x00, 0x02, 0x02, 0x00, 0x02, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(invalid_msg, sizeof(invalid_msg));
	RLM3_GPS_Init();

	const RLM3_GPS_MESSAGE* msg = RLM3_GPS_GetNextMessage(10);

	ASSERT(g_errors.size() == 1);
	ASSERT(g_errors[0] == RLM3_GPS_ERROR_PROTOCOL_FAIL);
	ASSERT(msg == nullptr);
}

TEST_CASE(RLM3_GPS_Error_ProtocolFail_B)
{
	g_errors.clear();
	uint8_t invalid_msg[] = { 0xA0, 0xA2, 0x00, 0x02, 0x02, 0x00, 0x02, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(invalid_msg, sizeof(invalid_msg));
	RLM3_GPS_Init();

	const RLM3_GPS_MESSAGE* msg = RLM3_GPS_GetNextMessage(10);

	ASSERT(g_errors.size() == 1);
	ASSERT(g_errors[0] == RLM3_GPS_ERROR_PROTOCOL_FAIL);
	ASSERT(msg == nullptr);
}

TEST_CASE(RLM3_GPS_Error_ProtocolFail_C)
{
	g_errors.clear();
	uint8_t invalid_msg[] = { 0xA0, 0xA1, 0x00, 0x02, 0x02, 0x00, 0x02, 0x0E, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(invalid_msg, sizeof(invalid_msg));
	RLM3_GPS_Init();

	const RLM3_GPS_MESSAGE* msg = RLM3_GPS_GetNextMessage(10);

	ASSERT(g_errors.size() == 1);
	ASSERT(g_errors[0] == RLM3_GPS_ERROR_PROTOCOL_FAIL);
	ASSERT(msg == nullptr);
}

TEST_CASE(RLM3_GPS_Error_ProtocolFail_D)
{
	g_errors.clear();
	uint8_t invalid_msg[] = { 0xA0, 0xA1, 0x00, 0x02, 0x02, 0x00, 0x02, 0x0D, 0x0D };
	SIM_RLM3_UART2_ReceiveRaw(invalid_msg, sizeof(invalid_msg));
	RLM3_GPS_Init();

	const RLM3_GPS_MESSAGE* msg = RLM3_GPS_GetNextMessage(10);

	ASSERT(g_errors.size() == 1);
	ASSERT(g_errors[0] == RLM3_GPS_ERROR_PROTOCOL_FAIL);
	ASSERT(msg == nullptr);
}

TEST_CASE(RLM3_GPS_Error_ProtocolFail_E)
{
	g_errors.clear();
	uint8_t invalid_msg[] = { 0xA0, 0xA1, 0x00, 0x00, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(invalid_msg, sizeof(invalid_msg));
	RLM3_GPS_Init();

	const RLM3_GPS_MESSAGE* msg = RLM3_GPS_GetNextMessage(10);

	ASSERT(g_errors.size() == 1);
	ASSERT(g_errors[0] == RLM3_GPS_ERROR_PROTOCOL_FAIL);
	ASSERT(msg == nullptr);
}

TEST_CASE(RLM3_GPS_ERROR_BufferFull)
{
	g_errors.clear();
	uint8_t simple_msg[] = { 0xA0, 0xA1, 0x00, 0x02, 0x02, 0x00, 0x02, 0x0D, 0x0A };
	for (size_t i = 0; i < 1024; i+= 4)
		SIM_RLM3_UART2_ReceiveRaw(simple_msg, sizeof(simple_msg));
	SIM_AddDelay(1234);
	SIM_RLM3_UART2_ReceiveRaw(simple_msg, sizeof(simple_msg));
	RLM3_GPS_Init();
	RLM3_Time start_time = RLM3_GetCurrentTime();

	while (g_errors.empty())
		RLM3_Take();

	ASSERT(RLM3_GetCurrentTime() == 1234 + start_time);
	ASSERT(g_errors.size() == 1);
	ASSERT(g_errors[0] == RLM3_GPS_ERROR_BUFFER_FULL);
}

TEST_CASE(RLM3_GPS_ParseMessage01)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x0F, 0x01, 0x01, 0xD6, 0x07, 0x0C, 0x12, 0x08, 0x32, 0x29, 0xC4, 0x09, 0x70, 0x30, 0x64, 0x00, 0x35, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_01_SYSTEM_RESTART*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 15);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_01_SYSTEM_RESTART);
	ASSERT(message->start_mode == 1);
	ASSERT(message->utc_year == ::hton_u16(0xD607));
	ASSERT(message->utc_month == 0x0C);
	ASSERT(message->utc_day == 0x12);
	ASSERT(message->utc_hour == 0x08);
	ASSERT(message->utc_minute == 0x32);
	ASSERT(message->utc_second == 0x29);
	ASSERT(message->latitude == ::hton_u16(0xC409));
	ASSERT(message->longitude == ::hton_u16(0x7030));
	ASSERT(message->altitude == ::hton_u16(0x6400));
}

TEST_CASE(RLM3_GPS_ParseMessage02)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x02, 0x02, 0x00, 0x02, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_02_QUERY_SOFTWARE_VERSION*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 2);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_02_QUERY_SOFTWARE_VERSION);
	ASSERT(message->software_type == 0x00);
}

TEST_CASE(RLM3_GPS_ParseMessage03)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x02, 0x03, 0x00, 0x03, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_03_QUERY_SOFTWARE_CRC*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 2);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_03_QUERY_SOFTWARE_CRC);
	ASSERT(message->software_type == 0x00);
}

TEST_CASE(RLM3_GPS_ParseMessage04)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x02, 0x04, 0x00, 0x04, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_04_SET_FACTORY_DEFAULTS*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 2);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_04_SET_FACTORY_DEFAULTS);
	ASSERT(message->type == 0x00);
}

TEST_CASE(RLM3_GPS_ParseMessage05)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x04, 0x05, 0x00, 0x00, 0x00, 0x05, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_05_CONFIGURE_SERIAL_PORT*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 4);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_05_CONFIGURE_SERIAL_PORT);
	ASSERT(message->com_port == 0x00);
	ASSERT(message->baud_rate == 0x00);
	ASSERT(message->attributes == 0x00);
}

TEST_CASE(RLM3_GPS_ParseMessage08)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x09, 0x08, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x08, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_08_CONFIGURE_NMEA_MESSAGE*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == 9);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_08_CONFIGURE_NMEA_MESSAGE);
	ASSERT(message->gga_interval == 0x01);
	ASSERT(message->gsa_interval == 0x01);
	ASSERT(message->gsv_interval == 0x01);
	ASSERT(message->gll_interval == 0x00);
	ASSERT(message->rmc_interval == 0x01);
	ASSERT(message->vtg_interval == 0x00);
	ASSERT(message->zda_interval == 0x00);
	ASSERT(message->attributes == 0x00);
}

TEST_CASE(RLM3_GPS_ParseMessage09)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x03, 0x09, 0x00, 0x00, 0x09, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_09_CONFIGURE_OUTPUT_MESSAGE_FORMAT*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 3);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_09_CONFIGURE_OUTPUT_MESSAGE_FORMAT);
	ASSERT(message->type == 0x00);
	ASSERT(message->attributes == 0x00);
}

TEST_CASE(RLM3_GPS_ParseMessage0E)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x03, 0x0E, 0x01, 0x00, 0x0F, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_0E_CONFIGURE_SYSTEM_POSITION_RATE*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 3);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_0E_CONFIGURE_SYSTEM_POSITION_RATE);
	ASSERT(message->rate == 0x01);
	ASSERT(message->attributes == 0x00);
}

TEST_CASE(RLM3_GPS_ParseMessage10)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x01, 0x10, 0x10, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_10_QUERY_SYSTEM_POSITION_UPDATE_RATE*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 1);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_10_QUERY_SYSTEM_POSITION_UPDATE_RATE);
}

TEST_CASE(RLM3_GPS_ParseMessage1E)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x09, 0x1E, 0x00, 0x01, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00, 0x1F, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_1E_CONFIGURE_BINARY_MEASUREMENT_DATA_OUTPUT*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 9);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_1E_CONFIGURE_BINARY_MEASUREMENT_DATA_OUTPUT);
	ASSERT(message->output_rate == 0x00);
	ASSERT(message->meas_time_enabling == 0x01);
	ASSERT(message->raw_meas_enabling == 0x01);
	ASSERT(message->sv_ch_status_enabling == 0x01);
	ASSERT(message->rcv_state_enabling == 0x00);
	ASSERT(message->subframe_enabling == 0x01);
	ASSERT(message->attributes == 0x01);
	ASSERT(message->reserved == 0);
}

TEST_CASE(RLM3_GPS_ParseMessage1F)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x01, 0x1F, 0x1F, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_1F_QUERY_BINARY_MEASUREMENT_DATA_OUTPUT_STATUS*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 1);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_1F_QUERY_BINARY_MEASUREMENT_DATA_OUTPUT_STATUS);
}

TEST_CASE(RLM3_GPS_ParseMessage30)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x02, 0x30, 0x00, 0x30, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_30_GET_GPS_EPHEMERIS*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 2);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_30_GET_GPS_EPHEMERIS);
	ASSERT(message->sv_id == 0x00);
}

TEST_CASE(RLM3_GPS_ParseMessage31)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x57, 0x31, 0x00, 0x02, 0x00, 0x77, 0x88, 0x04, 0x61, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDB, 0xDF, 0x59, 0xA6, 0x00, 0x00, 0x1E, 0x0A, 0x47, 0x7C, 0x00, 0x77, 0x88, 0x88, 0xDF, 0xFD, 0x2E, 0x35, 0xA9, 0xCD, 0xB0, 0xF0, 0x9F, 0xFD, 0xA7, 0x04, 0x8E, 0xCC, 0xA8, 0x10, 0x2C, 0xA1, 0x0E, 0x22, 0x31, 0x59, 0xA6, 0x74, 0x00, 0x77, 0x89, 0x0C, 0xFF, 0xA3, 0x59, 0x86, 0xC7, 0x77, 0xFF, 0xF8, 0x26, 0x97, 0xE3, 0xB9, 0x1C, 0x60, 0x59, 0xC3, 0x07, 0x44, 0xFF, 0xA6, 0x37, 0xDF, 0xF0, 0xB0, 0x5E, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_31_SET_EPHEMERIS*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 87);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_31_SET_EPHEMERIS);
	ASSERT(message->sv_id == ::ntoh_u16(0x0002));
}

TEST_CASE(RLM3_GPS_ParseMessage5B)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x02, 0x5B, 0x00, 0x5B, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_5B_GET_GLONASS_EPHEMERIS*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 2);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_5B_GET_GLONASS_EPHEMERIS);
	ASSERT(message->sv_id == 0x00);
}

TEST_CASE(RLM3_GPS_ParseMessage5C)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x2B, 0x5C, 0x02, 0xFC, 0x01, 0x02, 0x57, 0x07, 0x56, 0x1C, 0x9D, 0x2F, 0xE6, 0x84, 0x02, 0x12, 0x60, 0x99, 0x5C, 0xB8, 0x0A, 0x7A, 0x7D, 0x33, 0x03, 0x80, 0x26, 0x30, 0xC3, 0x9B, 0xA1, 0x78, 0x6A, 0x18, 0x04, 0x83, 0x4C, 0x84, 0xC0, 0x00, 0x02, 0xA1, 0x6D, 0x89, 0xF6, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_5C_SET_GLONASS_EPHEMERIS*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 43);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_5C_SET_GLONASS_EPHEMERIS);
	ASSERT(message->slot_number == 0x02);
	ASSERT(message->k_number == 0xFC);
	ASSERT(message->glo_eph_data[0][0] == 0x01);
	ASSERT(message->glo_eph_data[0][1] == 0x02);
	ASSERT(message->glo_eph_data[0][2] == 0x57);
	ASSERT(message->glo_eph_data[0][3] == 0x07);
	ASSERT(message->glo_eph_data[0][4] == 0x56);
	ASSERT(message->glo_eph_data[0][5] == 0x1C);
	ASSERT(message->glo_eph_data[0][6] == 0x9D);
	ASSERT(message->glo_eph_data[0][7] == 0x2F);
	ASSERT(message->glo_eph_data[0][8] == 0xE6);
	ASSERT(message->glo_eph_data[0][9] == 0x84);
	ASSERT(message->glo_eph_data[1][0] == 0x02);
	ASSERT(message->glo_eph_data[1][1] == 0x12);
	ASSERT(message->glo_eph_data[1][2] == 0x60);
	ASSERT(message->glo_eph_data[1][3] == 0x99);
	ASSERT(message->glo_eph_data[1][4] == 0x5C);
	ASSERT(message->glo_eph_data[1][5] == 0xB8);
	ASSERT(message->glo_eph_data[1][6] == 0x0A);
	ASSERT(message->glo_eph_data[1][7] == 0x7A);
	ASSERT(message->glo_eph_data[1][8] == 0x7D);
	ASSERT(message->glo_eph_data[1][9] == 0x33);
}

TEST_CASE(RLM3_GPS_ParseMessage80)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x0E, 0x80, 0x01, 0x00, 0x01, 0x01, 0x01, 0x00, 0x01, 0x03, 0x0E, 0x00, 0x07, 0x01, 0x12, 0x98, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_80_SOFTWARE_VERSION*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 14);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_80_SOFTWARE_VERSION);
	ASSERT(message->software_type == 0x01);
	ASSERT(message->kernel_version == ::ntoh_u32(0x00010101));
	ASSERT(message->odm_version == ::ntoh_u32(0x0001030E));
	ASSERT(message->revision == ::ntoh_u32(0x00070112));
}

TEST_CASE(RLM3_GPS_ParseMessage81)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x04, 0x81, 0x01, 0x98, 0x76, 0x6E, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_81_SOFTWARE_CRC*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 4);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_81_SOFTWARE_CRC);
	ASSERT(message->software_type == 0x01);
	ASSERT(message->crc == ::ntoh_u16(0x9876));
}

TEST_CASE(RLM3_GPS_ParseMessage83)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x02, 0x83, 0x02, 0x81, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_83_ACK*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 2);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_83_ACK);
	ASSERT(message->ack_id == 0x02);
}

TEST_CASE(RLM3_GPS_ParseMessage84)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x02, 0x84, 0x01, 0x85, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_84_NACK*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 2);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_84_NACK);
	ASSERT(message->nack_id == 0x01);
}

TEST_CASE(RLM3_GPS_ParseMessage86)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x02, 0x86, 0x01, 0x87, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_86_POSITION_UPDATE_RATE*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 2);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_86_POSITION_UPDATE_RATE);
	ASSERT(message->update_rate == 0x01);
}

TEST_CASE(RLM3_GPS_ParseMessage89)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x07, 0x89, 0x00, 0x01, 0x01, 0x01, 0x00, 0x01, 0x89, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_89_BINARY_MEASUREMENT_DATA_OUTPUT_STATUS*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 7);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_89_BINARY_MEASUREMENT_DATA_OUTPUT_STATUS);
	ASSERT(message->output_rate == 0x00);
	ASSERT(message->meas_time_enabling == 0x01);
	ASSERT(message->raw_meas_enabling == 0x01);
	ASSERT(message->sv_ch_status_enabling == 0x01);
	ASSERT(message->rcv_state_enabling == 0x00);
	ASSERT(message->subframe_enabling == 0x01);
}

TEST_CASE(RLM3_GPS_ParseMessage90)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x2B, 0x90, 0x02, 0xFC, 0x01, 0x02, 0xD2, 0x81, 0xF4, 0x75, 0x05, 0x16, 0x51, 0x9A, 0x02, 0x12, 0xE0, 0xAD, 0x0F, 0x37, 0x01, 0x7A, 0xD2, 0x06, 0x03, 0x80, 0x26, 0x19, 0xA1, 0x22, 0xA2, 0x84, 0xEB, 0xD6, 0x04, 0x83, 0x4C, 0xA8, 0xC0, 0x00, 0x02, 0xA1, 0x6D, 0x89, 0x6D, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_90_GLONASS_EPHEMERIS_DATA*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 43);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_90_GLONASS_EPHEMERIS_DATA);
	ASSERT(message->slot_number == 0x02);
	ASSERT(message->k_number == 0xFC);
	ASSERT(message->glo_eph_data[0][0] == 0x01);
	ASSERT(message->glo_eph_data[0][1] == 0x02);
	ASSERT(message->glo_eph_data[0][2] == 0xD2);
	ASSERT(message->glo_eph_data[0][3] == 0x81);
	ASSERT(message->glo_eph_data[0][4] == 0xF4);
	ASSERT(message->glo_eph_data[0][5] == 0x75);
	ASSERT(message->glo_eph_data[0][6] == 0x05);
	ASSERT(message->glo_eph_data[0][7] == 0x16);
	ASSERT(message->glo_eph_data[0][8] == 0x51);
	ASSERT(message->glo_eph_data[0][9] == 0x9A);
	ASSERT(message->glo_eph_data[1][0] == 0x02);
	ASSERT(message->glo_eph_data[1][1] == 0x12);
	ASSERT(message->glo_eph_data[1][2] == 0xE0);
	ASSERT(message->glo_eph_data[1][3] == 0xAD);
	ASSERT(message->glo_eph_data[1][4] == 0x0F);
	ASSERT(message->glo_eph_data[1][5] == 0x37);
	ASSERT(message->glo_eph_data[1][6] == 0x01);
	ASSERT(message->glo_eph_data[1][7] == 0x7A);
	ASSERT(message->glo_eph_data[1][8] == 0xD2);
	ASSERT(message->glo_eph_data[1][9] == 0x06);
}

TEST_CASE(RLM3_GPS_ParseMessageB1)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x57, 0xB1, 0x00, 0x02, 0x00, 0x77, 0x88, 0x04, 0x61, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDB, 0xDF, 0x59, 0xA6, 0x00, 0x00, 0x1E, 0x0A, 0x47, 0x7C, 0x00, 0x77, 0x88, 0x88, 0xDF, 0xFD, 0x2E, 0x35, 0xA9, 0xCD, 0xB0, 0xF0, 0x9F, 0xFD, 0xA7, 0x04, 0x8E, 0xCC, 0xA8, 0x10, 0x2C, 0xA1, 0x0E, 0x22, 0x31, 0x59, 0xA6, 0x74, 0x00, 0x77, 0x89, 0x0C, 0xFF, 0xA3, 0x59, 0x86, 0xC7, 0x77, 0xFF, 0xF8, 0x26, 0x97, 0xE3, 0xB9, 0x1C, 0x60, 0x59, 0xC3, 0x07, 0x44, 0xFF, 0xA6, 0x37, 0xDF, 0xF0, 0xB0, 0xDE, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_B1_GPS_EPHEMERIS_DATA*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 87);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_B1_GPS_EPHEMERIS_DATA);
	ASSERT(message->sv_id == ::ntoh_u16(0x02));
}

TEST_CASE(RLM3_GPS_ParseMessageDC)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x0A, 0xDC, 0x3D, 0x06, 0xED, 0x0B, 0x0C, 0xBC, 0x40, 0x03, 0xE8, 0x1A, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_DC_MEASUREMENT_TIME*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 10);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_DC_MEASUREMENT_TIME);
	ASSERT(message->iod == 0x3D);
	ASSERT(message->receiver_wn == ::ntoh_u16(0x06ED));
	ASSERT(message->receiver_tow == ::ntoh_u32(0x0B0CBC40));
	ASSERT(message->measurement_period == ::ntoh_u16(0x03E8));
}

static float ntoh_f32_u(uint32_t x)
{
	return ntoh_f32(*(float*)&x);
}

static double ntoh_f64_u(uint64_t x)
{
	return ntoh_f64(*(double*)&x);
}

TEST_CASE(RLM3_GPS_ParseMessageDD)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x01, 0x5C, 0xDD, 0x3D, 0x0F, 0x02, 0x2B, 0x41, 0x74, 0x42, 0xDB, 0x76, 0x55, 0xFA, 0x29, 0xC0, 0xE2, 0xE4, 0x02, 0x21, 0x5A, 0x00, 0x00, 0x44, 0x20, 0x80, 0x00, 0x07, 0x09, 0x29, 0x41, 0x77, 0x8C, 0xF0, 0xA9, 0xE7, 0x0C, 0x43, 0xC0, 0xF9, 0x72, 0x54, 0x2E, 0xEB, 0x80, 0x00, 0x44, 0xE3, 0xA0, 0x00, 0x07, 0x0A, 0x28, 0x41, 0x75, 0xCA, 0x96, 0x91, 0xA9, 0xE9, 0x23, 0x41, 0x04, 0x7D, 0xB1, 0xE9, 0xA9, 0x80, 0x00, 0xC5, 0x31, 0x20, 0x00, 0x07, 0x05, 0x2B, 0x41, 0x74, 0x9E, 0xBE, 0xEE, 0x17, 0x8C, 0x6A, 0x40, 0xD3, 0x71, 0xD4, 0x80, 0xCF, 0x00, 0x00, 0xC3, 0xAE, 0x00, 0x00, 0x07, 0x1A, 0x2E, 0x41, 0x75, 0x02, 0x83, 0xE5, 0xEC, 0xD7, 0x65, 0xC1, 0x04, 0x6D, 0x73, 0xBD, 0xE6, 0x20, 0x00, 0x45, 0x33, 0x30, 0x00, 0x07, 0x0C, 0x28, 0x41, 0x77, 0xC1, 0xE0, 0x1D, 0xA7, 0x2E, 0xC1, 0x40, 0xFF, 0x79, 0x4C, 0xC9, 0x14, 0x80, 0x00, 0xC5, 0x0D, 0x80, 0x00, 0x07, 0x11, 0x28, 0x41, 0x77, 0xE7, 0xB0, 0xE8, 0x15, 0x9A, 0xA8, 0x41, 0x0C, 0x87, 0x99, 0x0C, 0xFA, 0xA0, 0x00, 0xC5, 0x80, 0xD8, 0x00, 0x07, 0x0F, 0x27, 0x41, 0x77, 0x93, 0x96, 0x77, 0x03, 0x2B, 0x0A, 0xC1, 0x06, 0xBF, 0x2C, 0x49, 0x05, 0x60, 0x00, 0x45, 0x4F, 0xB0, 0x00, 0x07, 0x04, 0x2C, 0x41, 0x75, 0xBA, 0x4E, 0xB0, 0x68, 0x2B, 0x43, 0x40, 0xFB, 0x25, 0xC7, 0xA3, 0xB6, 0xC0, 0x00, 0xC4, 0xFE, 0x60, 0x00, 0x07, 0x07, 0x26, 0x41, 0x78, 0x48, 0x7F, 0x72, 0xDF, 0xC5, 0x81, 0xC0, 0xD0, 0x89, 0xC8, 0xBF, 0x96, 0x00, 0x00, 0x43, 0xA7, 0x80, 0x00, 0x07, 0x0D, 0x1D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x05, 0xF9, 0xA2, 0xD6, 0x0D, 0x40, 0x00, 0xC5, 0x66, 0x00, 0x00, 0x16, 0x08, 0x27, 0x41, 0x78, 0x6A, 0xD7, 0xA4, 0x71, 0x2A, 0x50, 0xC0, 0xEF, 0x02, 0x44, 0x2E, 0x09, 0x80, 0x00, 0x44, 0xA2, 0x80, 0x00, 0x07, 0x19, 0x23, 0x41, 0x78, 0x7E, 0xE4, 0x8B, 0x0C, 0x9E, 0x26, 0x40, 0xE6, 0xAD, 0x04, 0x2B, 0x85, 0x80, 0x00, 0xC4, 0x98, 0x20, 0x00, 0x07, 0x42, 0x1F, 0x41, 0x75, 0x27, 0xEA, 0xE2, 0x16, 0x7D, 0x10, 0x41, 0x06, 0xD6, 0x0A, 0x57, 0x6B, 0x00, 0x00, 0xC5, 0x53, 0x10, 0x00, 0x07, 0x52, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xFE, 0x83, 0x49, 0x5D, 0xA7, 0x00, 0x00, 0x45, 0x16, 0xC0, 0x00, 0x06, 0xAA, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_DD_RAW_MEASUREMENT*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == 3 + message->nmeas * 23);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_DD_RAW_MEASUREMENT);
	ASSERT(message->iod == 0x3D);
	ASSERT(message->nmeas == 0x0F);
	ASSERT(message->channels[0].svid == 0x02);
	ASSERT(message->channels[0].cn0 == 0x2B);
	ASSERT(message->channels[0].pseudo_range == ::ntoh_f64_u(0x417442DB7655FA29));
	ASSERT(message->channels[0].accumulated_carrier_cycles == ::ntoh_f64_u(0xC0E2E402215A0000));
	ASSERT(message->channels[0].doppler_frequency == ::ntoh_f32_u(0x44208000));
	ASSERT(message->channels[0].measurement_indicator == 0x07);
}

TEST_CASE(RLM3_GPS_ParseMessageDE)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0xA3, 0xDE, 0x3D, 0x10, 0x00, 0x02, 0x07, 0x01, 0x2B, 0x00, 0x3E, 0x00, 0x10, 0x1F, 0x01, 0x09, 0x07, 0x01, 0x29, 0x00, 0x10, 0x00, 0x72, 0x1F, 0x02, 0x0A, 0x07, 0x01, 0x28, 0x00, 0x22, 0x00, 0x27, 0x1F, 0x03, 0x05, 0x07, 0x00, 0x2B, 0x00, 0x38, 0x01, 0x38, 0x1F, 0x04, 0x1A, 0x07, 0x00, 0x2E, 0x00, 0x2E, 0x00, 0xBA, 0x1F, 0x05, 0x0C, 0x07, 0x00, 0x28, 0x00, 0x0E, 0x00, 0xF8, 0x1F, 0x06, 0x11, 0x07, 0x01, 0x28, 0x00, 0x0A, 0x00, 0x9A, 0x1F, 0x07, 0x0F, 0x07, 0x00, 0x27, 0x00, 0x0E, 0x00, 0xD1, 0x1F, 0x08, 0x21, 0x07, 0x00, 0x29, 0x00, 0x42, 0x00, 0x2E, 0x1F, 0x09, 0x04, 0x07, 0x00, 0x2C, 0x00, 0x26, 0x00, 0x5B, 0x1F, 0x0C, 0x07, 0x07, 0x00, 0x26, 0x00, 0x09, 0x00, 0x4D, 0x1F, 0x0D, 0x0D, 0x07, 0x00, 0x1D, 0x00, 0x06, 0x00, 0x24, 0x1F, 0x0E, 0x08, 0x07, 0x00, 0x27, 0x00, 0x0A, 0x00, 0x6B, 0x1F, 0x0F, 0x19, 0x07, 0x00, 0x23, 0x00, 0x06, 0x01, 0x1B, 0x1F, 0x10, 0x42, 0x06, 0x05, 0x1F, 0x00, 0x20, 0x00, 0x15, 0x1F, 0x11, 0x52, 0x07, 0x05, 0x1E, 0x00, 0x31, 0x01, 0x4E, 0x1F, 0xC7, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_DE_SV_CH_STATUS*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == 3 + message->nsvs * 10);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_DE_SV_CH_STATUS);
	ASSERT(message->iod == 0x3D);
	ASSERT(message->nsvs == 0x10);
	ASSERT(message->channels[0].channel_id == 0x00);
	ASSERT(message->channels[0].svid == 0x02);
	ASSERT(message->channels[0].sv_status_indicator == 0x07);
	ASSERT(message->channels[0].ura == 0x01);
	ASSERT(message->channels[0].cn0 == 0x2B);
	ASSERT(message->channels[0].elevation == ::ntoh_u16(0x003E));
	ASSERT(message->channels[0].azimoth == ::ntoh_u16(0x10));
	ASSERT(message->channels[0].channel_status_indicator == 0x1F);
}

TEST_CASE(RLM3_GPS_ParseMessageDF)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x51, 0xDF, 0x92, 0x03, 0x06, 0xED, 0x41, 0x07, 0xDB, 0xE7, 0xFD, 0x76, 0x3B, 0x21, 0xC1, 0x46, 0xC6, 0x04, 0x2F, 0x62, 0xBF, 0xD8, 0x41, 0x52, 0xF1, 0xB6, 0x4B, 0x17, 0xF7, 0xCC, 0x41, 0x44, 0x46, 0x79, 0xB8, 0x7A, 0xDB, 0x12, 0x3C, 0x8A, 0xAA, 0xD4, 0xBC, 0x1A, 0x6E, 0xF0, 0xBB, 0xC5, 0x67, 0xD2, 0x41, 0x16, 0xAD, 0x5E, 0x6D, 0x3F, 0x7C, 0x78, 0x42, 0x8F, 0xD9, 0x1E, 0x40, 0x5D, 0x7C, 0x6B, 0x40, 0x4B, 0x07, 0xFB, 0x3F, 0x7C, 0x51, 0xAD, 0x40, 0x40, 0xFB, 0xC2, 0x3F, 0xB1, 0x06, 0x30, 0x23, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_DF_RECEIVER_STATE*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 81);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_DF_RECEIVER_STATE);
	ASSERT(message->iod == 0x92);
	ASSERT(message->navigation_state == 0x03);
	ASSERT(message->wn == ::ntoh_u16(0x06ED));
	ASSERT(message->tow == ::ntoh_f64_u(0x4107DBE7FD763B21));
	ASSERT(message->ecef_pos_x == ::ntoh_f64_u(0xC146C6042F62BFD8));
	ASSERT(message->ecef_pos_y == ::ntoh_f64_u(0x4152F1B64B17F7CC));
	ASSERT(message->ecef_pos_z == ::ntoh_f64_u(0x41444679B87ADB12));
	ASSERT(message->ecef_vel_x == ::ntoh_f32_u(0x3C8AAAD4));
	ASSERT(message->ecef_vel_y == ::ntoh_f32_u(0xBC1A6EF0));
	ASSERT(message->ecef_vel_z == ::ntoh_f32_u(0xBBC567D2));
	ASSERT(message->clock_bias == ::ntoh_f64_u(0x4116AD5E6D3F7C78));
	ASSERT(message->clock_drift == ::ntoh_f32_u(0x428FD91E));
	ASSERT(message->gdop == ::ntoh_f32_u(0x405D7C6B));
	ASSERT(message->pdop == ::ntoh_f32_u(0x404B07FB));
	ASSERT(message->hdop == ::ntoh_f32_u(0x3F7C51AD));
	ASSERT(message->vdop == ::ntoh_f32_u(0x4040FBC2));
	ASSERT(message->tdop == ::ntoh_f32_u(0x3FB10630));
}

TEST_CASE(RLM3_GPS_ParseMessageE0)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x21, 0xE0, 0x02, 0x05, 0x8B, 0x0B, 0xB4, 0x3F, 0x22, 0xB5, 0x4F, 0x31, 0xCF, 0x4E, 0xFD, 0x81, 0xFD, 0x4D, 0x00, 0xA1, 0x0C, 0x98, 0x79, 0xE7, 0x09, 0x08, 0xD5, 0xC5, 0xF8, 0xED, 0x03, 0xEB, 0xFF, 0xF4, 0x04, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_E0_GPS_SUBFRAME*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 33);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_E0_GPS_SUBFRAME);
	ASSERT(message->svid == 0x02);
	ASSERT(message->sfid == 0x05);
	uint8_t raw_subframe[30] = { 0x8B, 0x0B, 0xB4, 0x3F, 0x22, 0xB5, 0x4F, 0x31, 0xCF, 0x4E, 0xFD, 0x81, 0xFD, 0x4D, 0x00, 0xA1, 0x0C, 0x98, 0x79, 0xE7, 0x09, 0x08, 0xD5, 0xC5, 0xF8, 0xED, 0x03, 0xEB, 0xFF, 0xF4 };
	for (size_t i = 0; i < 30; i++)
		ASSERT(message->subframe[i] == raw_subframe[i]);
}

TEST_CASE(RLM3_GPS_ParseMessageE1)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x0C, 0xE1, 0x52, 0x0E, 0xB4, 0x05, 0xA9, 0xC3, 0x94, 0x17, 0x50, 0x04, 0x82, 0x33, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_E1_GLONASS_STRING_BUFFER*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 12);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_E1_GLONASS_STRING_BUFFER);
	ASSERT(message->svid == 0x52);
	ASSERT(message->string_number == 0x0E);
	ASSERT(message->data[0] == 0xB4);
	ASSERT(message->data[1] == 0x05);
	ASSERT(message->data[2] == 0xA9);
	ASSERT(message->data[3] == 0xC3);
	ASSERT(message->data[4] == 0x94);
	ASSERT(message->data[5] == 0x17);
	ASSERT(message->data[6] == 0x50);
	ASSERT(message->data[7] == 0x04);
	ASSERT(message->data[8] == 0x82);
}

TEST_CASE(RLM3_GPS_ParseMessageE2)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x1F, 0xE2, 0xCF, 0x01, 0xE2, 0x40, 0x47, 0x37, 0x58, 0x00, 0x0D, 0xA0, 0xE1, 0x00, 0xAC, 0x03, 0x87, 0x8E, 0x31, 0x5B, 0x53, 0xB4, 0x12, 0xB2, 0xC0, 0x02, 0x5B, 0x04, 0x60, 0x07, 0xAB, 0x81, 0xB1, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_E2_BEIDOU2_D1_SUBFRAME_BUFFER*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 31);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_E2_BEIDOU2_D1_SUBFRAME_BUFFER);
	ASSERT(message->svid == 0xCF);
	ASSERT(message->sfid == 0x01);
	uint8_t raw_data[28] = { 0xE2, 0x40, 0x47, 0x37, 0x58, 0x00, 0x0D, 0xA0, 0xE1, 0x00, 0xAC, 0x03, 0x87, 0x8E, 0x31, 0x5B, 0x53, 0xB4, 0x12, 0xB2, 0xC0, 0x02, 0x5B, 0x04, 0x60, 0x07, 0xAB, 0x81 };
	for (size_t i = 0; i < 28; i++)
		ASSERT(message->data[i] == raw_data[i]);
}

TEST_CASE(RLM3_GPS_ParseMessageE3)
{
	uint8_t raw_message[] = { 0xA0, 0xA1, 0x00, 0x1F, 0xE3, 0xCB, 0x01, 0xE2, 0x40, 0x47, 0x37, 0x95, 0xA5, 0x14, 0xC8, 0xCA, 0xEA, 0xCF, 0xA5, 0x00, 0x15, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x48, 0x0D, 0x0A };
	SIM_RLM3_UART2_ReceiveRaw(raw_message, sizeof(raw_message));

	RLM3_GPS_Init();
	auto message = (const RLM3_GPS_MESSAGE_E3_BEIDOU2_D2_SUBFRAME_BUFFER*)RLM3_GPS_GetNextMessage(10);

	ASSERT(message != nullptr);
	ASSERT(message->payload_length == RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(*message));
	ASSERT(message->payload_length == 31);
	ASSERT(message->message_type == RLM3_GPS_MESSAGE_TYPE_E3_BEIDOU2_D2_SUBFRAME_BUFFER);
	ASSERT(message->svid == 0xCB);
	ASSERT(message->sfid == 0x01);
	uint8_t raw_data[28] = { 0xE2, 0x40, 0x47, 0x37, 0x95, 0xA5, 0x14, 0xC8, 0xCA, 0xEA, 0xCF, 0xA5, 0x00, 0x15, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55 };
	for (size_t i = 0; i < 28; i++)
		ASSERT(message->data[i] == raw_data[i]);
}
