#include "Test.hpp"
#include "rlm3-gps.h"
#include "rlm3-task.h"
#include "rlm3-bytes.h"
#include "logger.h"
#include <algorithm>


LOGGER_ZONE(TEST);


static bool SendAndVerify(const RLM3_GPS_MESSAGE* client_message, size_t timeout_ms);
static void Log(const RLM3_GPS_MESSAGE* client_message);


extern void RLM3_GPS_PulseCallback()
{
	LOG_ALWAYS("PULSE CALLBACK");
}

extern void RLM3_GPS_ErrorCallback(RLM3_GPS_Error error)
{
	LOG_ALWAYS("ERROR CALLBACK %d", error);
}


TEST_CASE(RLM3_GPS_Lifecycle_HappyCase)
{
	ASSERT(!RLM3_GPS_IsInit());
	RLM3_GPS_Init();
	ASSERT(RLM3_GPS_IsInit());
	RLM3_GPS_Deinit();
	ASSERT(!RLM3_GPS_IsInit());
}

TEST_CASE(RLM3_GPS_Run_HappyCase)
{
	RLM3_GPS_Init();

	RLM3_GPS_MESSAGE_02_QUERY_SOFTWARE_VERSION message02;
	RLM3_GPS_SET_MESSAGE_PAYLOAD_SIZE(message02);
	message02.message_type = RLM3_GPS_MESSAGE_TYPE_02_QUERY_SOFTWARE_VERSION;
	message02.software_type = 0;
	ASSERT(SendAndVerify((const RLM3_GPS_MESSAGE*)&message02, 1000));

	RLM3_GPS_MESSAGE_03_QUERY_SOFTWARE_CRC message03;
	RLM3_GPS_SET_MESSAGE_PAYLOAD_SIZE(message03);
	message03.message_type = RLM3_GPS_MESSAGE_TYPE_03_QUERY_SOFTWARE_CRC;
	message03.software_type = 0;
	ASSERT(SendAndVerify((const RLM3_GPS_MESSAGE*)&message03, 1000));

	RLM3_GPS_MESSAGE_10_QUERY_SYSTEM_POSITION_UPDATE_RATE message10;
	RLM3_GPS_SET_MESSAGE_PAYLOAD_SIZE(message10);
	message10.message_type = RLM3_GPS_MESSAGE_TYPE_10_QUERY_SYSTEM_POSITION_UPDATE_RATE;
	ASSERT(SendAndVerify((const RLM3_GPS_MESSAGE*)&message10, 1000));

	RLM3_GPS_MESSAGE_1F_QUERY_BINARY_MEASUREMENT_DATA_OUTPUT_STATUS message1F;
	RLM3_GPS_SET_MESSAGE_PAYLOAD_SIZE(message1F);
	message1F.message_type = RLM3_GPS_MESSAGE_TYPE_1F_QUERY_BINARY_MEASUREMENT_DATA_OUTPUT_STATUS;
	ASSERT(SendAndVerify((const RLM3_GPS_MESSAGE*)&message1F, 1000));

	RLM3_GPS_MESSAGE_09_CONFIGURE_OUTPUT_MESSAGE_FORMAT message09;
	RLM3_GPS_SET_MESSAGE_PAYLOAD_SIZE(message09);
	message09.message_type = RLM3_GPS_MESSAGE_TYPE_09_CONFIGURE_OUTPUT_MESSAGE_FORMAT;
	message09.type = 2; // Binary message
	message09.attributes = 0; // Update to SRAM
	ASSERT(SendAndVerify((const RLM3_GPS_MESSAGE*)&message09, 1000));

	RLM3_GPS_MESSAGE_1E_CONFIGURE_BINARY_MEASUREMENT_DATA_OUTPUT message1E;
	RLM3_GPS_SET_MESSAGE_PAYLOAD_SIZE(message1E);
	message1E.message_type = RLM3_GPS_MESSAGE_TYPE_1E_CONFIGURE_BINARY_MEASUREMENT_DATA_OUTPUT;
	message1E.output_rate = 0; // 1Hz
	message1E.meas_time_enabling = 1; // Enable
	message1E.raw_meas_enabling = 1; // Enable
	message1E.sv_ch_status_enabling = 1; // Enable
	message1E.rcv_state_enabling = 1; // Disable
	message1E.subframe_enabling = 0x00; // None
	message1E.attributes = 0; // Update to SRAM
	message1E.reserved = 0; // Reserved?
	ASSERT(SendAndVerify((const RLM3_GPS_MESSAGE*)&message1E, 1000));

	ASSERT(SendAndVerify((const RLM3_GPS_MESSAGE*)&message1F, 1000));

	bool had_measurement_time = false;
	bool had_raw_measurement = false;
	bool had_channel_status = false;
	RLM3_Time start_time = RLM3_GetCurrentTime();
	while (RLM3_GetCurrentTime() - start_time < 20000 && !(had_measurement_time && had_raw_measurement && had_channel_status))
	{
		const RLM3_GPS_MESSAGE* server_message = RLM3_GPS_GetNextMessage(10);
		if (server_message == nullptr)
			continue;

		Log(server_message);

		if (server_message->message_type == RLM3_GPS_MESSAGE_TYPE_DC_MEASUREMENT_TIME)
			had_measurement_time = true;

		if (server_message->message_type == RLM3_GPS_MESSAGE_TYPE_DD_RAW_MEASUREMENT)
			had_raw_measurement = true;

		if (server_message->message_type == RLM3_GPS_MESSAGE_TYPE_DE_SV_CH_STATUS)
			had_channel_status = true;
	}

	ASSERT(had_measurement_time && had_raw_measurement && had_channel_status);

	RLM3_GPS_Deinit();
}

static bool SendAndVerify(const RLM3_GPS_MESSAGE* client_message, size_t timeout_ms)
{
	Log(client_message);

	if (!RLM3_GPS_SendMessage((const RLM3_GPS_MESSAGE*)client_message))
	{
		LOG_ERROR("Failed sending message %x", client_message->message_type);
		return false;
	}

	RLM3_Time start_time = RLM3_GetCurrentTime();
	while (RLM3_GetCurrentTime() - start_time < timeout_ms)
	{
		const RLM3_GPS_MESSAGE* server_message = RLM3_GPS_GetNextMessage(10);
		if (server_message == nullptr)
			continue;

		Log(server_message);

		if (server_message->message_type == RLM3_GPS_MESSAGE_TYPE_83_ACK)
			return true;

		if (server_message->message_type == RLM3_GPS_MESSAGE_TYPE_84_NACK)
			return false;
	}

	return false;
}

static void Log(const RLM3_GPS_MESSAGE* message)
{
	if (message == nullptr)
	{
		LOG_ALWAYS("NULL");
		return;
	}

	switch (message->message_type)
	{
	case RLM3_GPS_MESSAGE_TYPE_01_SYSTEM_RESTART: { auto m = (const RLM3_GPS_MESSAGE_01_SYSTEM_RESTART*)message; LOG_ALWAYS("MESSAGE SYSTEM_RESTART { TYPE: 01, PAYLOAD_LENGTH: %d }", m->payload_length); } break;
	case RLM3_GPS_MESSAGE_TYPE_02_QUERY_SOFTWARE_VERSION: { auto m = (const RLM3_GPS_MESSAGE_02_QUERY_SOFTWARE_VERSION*)message; LOG_ALWAYS("MESSAGE QUERY_SOFTWARE_VERSION { TYPE: 02, PAYLOAD_LENGTH: %d }", m->payload_length); } break;
	case RLM3_GPS_MESSAGE_TYPE_03_QUERY_SOFTWARE_CRC: { auto m = (const RLM3_GPS_MESSAGE_03_QUERY_SOFTWARE_CRC*)message; LOG_ALWAYS("MESSAGE QUERY_SOFTWARE_CRC { TYPE: 03, PAYLOAD_LENGTH: %d }", m->payload_length); } break;
	case RLM3_GPS_MESSAGE_TYPE_04_SET_FACTORY_DEFAULTS: { auto m = (const RLM3_GPS_MESSAGE_04_SET_FACTORY_DEFAULTS*)message; LOG_ALWAYS("MESSAGE SET_FACTORY_DEFAULTS { TYPE: 04, PAYLOAD_LENGTH: %d }", m->payload_length); } break;
	case RLM3_GPS_MESSAGE_TYPE_05_CONFIGURE_SERIAL_PORT: { auto m = (const RLM3_GPS_MESSAGE_05_CONFIGURE_SERIAL_PORT*)message; LOG_ALWAYS("MESSAGE CONFIGURE_SERIAL_PORT { TYPE: 05, PAYLOAD_LENGTH: %d }", m->payload_length); } break;
	case RLM3_GPS_MESSAGE_TYPE_08_CONFIGURE_NMEA_MESSAGE: { auto m = (const RLM3_GPS_MESSAGE_08_CONFIGURE_NMEA_MESSAGE*)message; LOG_ALWAYS("MESSAGE CONFIGURE_NMEA_MESSAGE { TYPE: 08, PAYLOAD_LENGTH: %d }", m->payload_length); } break;
	case RLM3_GPS_MESSAGE_TYPE_09_CONFIGURE_OUTPUT_MESSAGE_FORMAT: { auto m = (const RLM3_GPS_MESSAGE_09_CONFIGURE_OUTPUT_MESSAGE_FORMAT*)message; LOG_ALWAYS("MESSAGE CONFIGURE_OUTPUT_MESSAGE_FORMAT { TYPE: 09, PAYLOAD_LENGTH: %d }", m->payload_length); } break;
	case RLM3_GPS_MESSAGE_TYPE_0E_CONFIGURE_SYSTEM_POSITION_RATE: { auto m = (const RLM3_GPS_MESSAGE_0E_CONFIGURE_SYSTEM_POSITION_RATE*)message; LOG_ALWAYS("MESSAGE CONFIGURE_SYSTEM_POSITION_RATE { TYPE: 0E, PAYLOAD_LENGTH: %d }", m->payload_length); } break;
	case RLM3_GPS_MESSAGE_TYPE_10_QUERY_SYSTEM_POSITION_UPDATE_RATE: { auto m = (const RLM3_GPS_MESSAGE_10_QUERY_SYSTEM_POSITION_UPDATE_RATE*)message; LOG_ALWAYS("MESSAGE QUERY_SYSTEM_POSITION_UPDATE_RATE { TYPE: 10, PAYLOAD_LENGTH: %d }", m->payload_length); } break;
	case RLM3_GPS_MESSAGE_TYPE_1E_CONFIGURE_BINARY_MEASUREMENT_DATA_OUTPUT:
	{
		auto m = (const RLM3_GPS_MESSAGE_1E_CONFIGURE_BINARY_MEASUREMENT_DATA_OUTPUT*)message;
		static const char* k_rates[] = { "1Hz", "2Hz", "4Hz", "5Hz", "10Hz", "20Hz", "<ERROR>" };
		static const char* k_enables[] = { "Disabled", "Enabled", "<ERROR>" };
		static const char* k_subframes[] = { "None", "GPS", "<OTHER or MULTIPLE>" };
		static const char* k_attributes[] = { "SDRAM", "SDRAM+FLASH", "<ERROR>" };
		const char* rate = k_rates[std::min<uint8_t>(6, m->output_rate)];
		const char* meas_time_enabling = k_enables[std::min<uint8_t>(2, m->meas_time_enabling)];
		const char* raw_meas_enabling = k_enables[std::min<uint8_t>(2, m->raw_meas_enabling)];
		const char* sv_ch_status_enabling = k_enables[std::min<uint8_t>(2, m->sv_ch_status_enabling)];
		const char* rcv_state_enabling = k_enables[std::min<uint8_t>(2, m->rcv_state_enabling)];
		const char* subframe_enabling = k_subframes[std::min<uint8_t>(2, m->subframe_enabling)];
		const char* attributes = k_attributes[std::min<uint8_t>(2, m->attributes)];
		LOG_ALWAYS("MESSAGE CONFIGURE_BINARY_MEASUREMENT_DATA_OUTPUT { TYPE: 1E, PAYLOAD_LENGTH: %d, RATE: %s, MEASURE: %s, RAW: %s, SV: %s, RCV: %s, SUBFRAMES: %s, ATTRIBUTES: %s, RESERVED: %d }", m->payload_length, rate, meas_time_enabling, raw_meas_enabling, sv_ch_status_enabling, rcv_state_enabling, subframe_enabling, attributes, m->reserved);
	}
	break;
	case RLM3_GPS_MESSAGE_TYPE_1F_QUERY_BINARY_MEASUREMENT_DATA_OUTPUT_STATUS: { auto m = (const RLM3_GPS_MESSAGE_1F_QUERY_BINARY_MEASUREMENT_DATA_OUTPUT_STATUS*)message; LOG_ALWAYS("MESSAGE QUERY_BINARY_MEASUREMENT_DATA_OUTPUT_STATUS { TYPE: 1F, PAYLOAD_LENGTH: %d }", m->payload_length); } break;
	case RLM3_GPS_MESSAGE_TYPE_30_GET_GPS_EPHEMERIS: { auto m = (const RLM3_GPS_MESSAGE_30_GET_GPS_EPHEMERIS*)message; LOG_ALWAYS("MESSAGE GET_GPS_EPHEMERIS { TYPE: 30, PAYLOAD_LENGTH: %d }", m->payload_length); } break;
	case RLM3_GPS_MESSAGE_TYPE_31_SET_EPHEMERIS: { auto m = (const RLM3_GPS_MESSAGE_31_SET_EPHEMERIS*)message; LOG_ALWAYS("MESSAGE SET_EPHEMERIS { TYPE: 31, PAYLOAD_LENGTH: %d }", m->payload_length); } break;
	case RLM3_GPS_MESSAGE_TYPE_5B_GET_GLONASS_EPHEMERIS: { auto m = (const RLM3_GPS_MESSAGE_5B_GET_GLONASS_EPHEMERIS*)message; LOG_ALWAYS("MESSAGE GET_GLONASS_EPHEMERIS { TYPE: 5B, PAYLOAD_LENGTH: %d }", m->payload_length); } break;
	case RLM3_GPS_MESSAGE_TYPE_5C_SET_GLONASS_EPHEMERIS: { auto m = (const RLM3_GPS_MESSAGE_5C_SET_GLONASS_EPHEMERIS*)message; LOG_ALWAYS("MESSAGE SET_GLONASS_EPHEMERIS { TYPE: 5C, PAYLOAD_LENGTH: %d }", m->payload_length); } break;
	case RLM3_GPS_MESSAGE_TYPE_80_SOFTWARE_VERSION:
	{
		auto m = (const RLM3_GPS_MESSAGE_80_SOFTWARE_VERSION*)message;
		LOG_ALWAYS("MESSAGE SOFTWARE_VERSION { TYPE: 80, PAYLOAD_LENGTH: %d, SOFTWARE_TYPE: %d, KERNEL_VERSION: %x, ODM_VERSION: %x, REVISION: %x }", m->payload_length, m->software_type, (int)ntoh(m->kernel_version), (int)ntoh(m->odm_version), (int)ntoh(m->revision));
	}
	break;
	case RLM3_GPS_MESSAGE_TYPE_81_SOFTWARE_CRC:
	{
		auto m = (const RLM3_GPS_MESSAGE_81_SOFTWARE_CRC*)message;
		LOG_ALWAYS("MESSAGE SOFTWARE_CRC { TYPE: 81, PAYLOAD_LENGTH: %d, SOFTWARE_TYPE: %d, CRC: %x }", m->payload_length, m->software_type, ntoh(m->crc));
	}
	break;
	case RLM3_GPS_MESSAGE_TYPE_83_ACK:
	{
		auto m = (const RLM3_GPS_MESSAGE_83_ACK*)message;
		LOG_ALWAYS("MESSAGE ACK { TYPE: 83, PAYLOAD_LENGTH: %d, ACK_ID: %x }", m->payload_length, m->ack_id);
	}
	break;
	case RLM3_GPS_MESSAGE_TYPE_84_NACK:
	{
		auto m = (const RLM3_GPS_MESSAGE_84_NACK*)message;
		LOG_ALWAYS("MESSAGE NACK { TYPE: 84, PAYLOAD_LENGTH: %d, NACK_ID: %x }", m->payload_length, m->nack_id);
	}
	break;
	case RLM3_GPS_MESSAGE_TYPE_86_POSITION_UPDATE_RATE:
	{
		auto m = (const RLM3_GPS_MESSAGE_86_POSITION_UPDATE_RATE*)message;
		LOG_ALWAYS("MESSAGE POSITION_UPDATE_RATE { TYPE: 86, PAYLOAD_LENGTH: %d, UPDATE_RATE: %d }", m->payload_length, m->update_rate);
	}
	break;
	case RLM3_GPS_MESSAGE_TYPE_89_BINARY_MEASUREMENT_DATA_OUTPUT_STATUS:
	{
		auto m = (const RLM3_GPS_MESSAGE_89_BINARY_MEASUREMENT_DATA_OUTPUT_STATUS*)message;
		static const char* k_rates[] = { "1Hz", "2Hz", "4Hz", "5Hz", "10Hz", "20Hz", "<ERROR>" };
		static const char* k_enables[] = { "Disabled", "Enabled", "<ERROR>" };
		static const char* k_subframes[] = { "None", "GPS", "<OTHER or MULTIPLE>" };
		const char* rate = k_rates[std::min<uint8_t>(6, m->output_rate)];
		const char* meas_time_enabling = k_enables[std::min<uint8_t>(2, m->meas_time_enabling)];
		const char* raw_meas_enabling = k_enables[std::min<uint8_t>(2, m->raw_meas_enabling)];
		const char* sv_ch_status_enabling = k_enables[std::min<uint8_t>(2, m->sv_ch_status_enabling)];
		const char* rcv_state_enabling = k_enables[std::min<uint8_t>(2, m->rcv_state_enabling)];
		const char* subframe_enabling = k_subframes[std::min<uint8_t>(2, m->subframe_enabling)];
		LOG_ALWAYS("MESSAGE BINARY_MEASUREMENT_DATA_OUTPUT_STATUS { TYPE: 89, PAYLOAD_LENGTH: %d, RATE: %s, MEASURE: %s, RAW: %s, SV: %s, RCV: %s, SUBFRAMES: %s }", m->payload_length, rate, meas_time_enabling, raw_meas_enabling, sv_ch_status_enabling, rcv_state_enabling, subframe_enabling);
	}
	break;
	case RLM3_GPS_MESSAGE_TYPE_90_GLONASS_EPHEMERIS_DATA: { auto m = (const RLM3_GPS_MESSAGE_90_GLONASS_EPHEMERIS_DATA*)message; LOG_ALWAYS("MESSAGE GLONASS_EPHEMERIS_DATA { TYPE: 90, PAYLOAD_LENGTH: %d }", m->payload_length); } break;
	case RLM3_GPS_MESSAGE_TYPE_B1_GPS_EPHEMERIS_DATA: { auto m = (const RLM3_GPS_MESSAGE_B1_GPS_EPHEMERIS_DATA*)message; LOG_ALWAYS("MESSAGE GPS_EPHEMERIS_DATA { TYPE: B1, PAYLOAD_LENGTH: %d }", m->payload_length); } break;
	case RLM3_GPS_MESSAGE_TYPE_DC_MEASUREMENT_TIME:
	{
		auto m = (const RLM3_GPS_MESSAGE_DC_MEASUREMENT_TIME*)message;
		LOG_ALWAYS("MESSAGE MEASUREMENT_TIME { TYPE: DC, PAYLOAD_LENGTH: %d, IOD: %d, WEEK: %d, TOW: %d, PERIOD: %d }", m->payload_length, m->iod, ntoh(m->receiver_wn), (int)ntoh(m->receiver_tow), ntoh(m->measurement_period));
	}
	break;
	case RLM3_GPS_MESSAGE_TYPE_DD_RAW_MEASUREMENT:
	{
		auto m = (const RLM3_GPS_MESSAGE_DD_RAW_MEASUREMENT*)message;
		LOG_ALWAYS("MESSAGE RAW_MEASUREMENT { TYPE: DD, PAYLOAD_LENGTH: %d, IOD: %d, NMEAS: %d }", m->payload_length, m->iod, m->nmeas);
		for (size_t i = 0; i < m->nmeas; i++)
		{
			auto r = m->channels[i];
			LOG_ALWAYS("- { SVID: %d, CN: %d, PR: %f, ACC: %f, DOP: %f, IND: %x }", r.svid, r.cn0, ntoh(r.pseudo_range), ntoh(r.accumulated_carrier_cycles), ntoh(r.doppler_frequency), r.measurement_indicator);
		}
	}
	break;
	case RLM3_GPS_MESSAGE_TYPE_DE_SV_CH_STATUS:
	{
		auto m = (const RLM3_GPS_MESSAGE_DE_SV_CH_STATUS*)message;
		LOG_ALWAYS("MESSAGE SV_CH_STATUS { TYPE: DE, PAYLOAD_LENGTH: %d, IOD: %d, NSVS: %d }", m->payload_length, m->iod, m->nsvs);
		for (size_t i = 0; i < m->nsvs; i++)
		{
			auto r = m->channels[i];
			LOG_ALWAYS("- { ID: %d, SVID: %d, SV_STATUS: %x, URA: %d, CN0: %d, ELEV: %d, AZ: %d, STATUS: %x }", r.channel_id, r.svid, r.sv_status_indicator, r.ura, r.cn0, ntoh(r.elevation), ntoh(r.azimoth), r.channel_status_indicator);
		}
	}
	break;
	case RLM3_GPS_MESSAGE_TYPE_DF_RECEIVER_STATE:
	{
		auto m = (const RLM3_GPS_MESSAGE_DF_RECEIVER_STATE*)message;
		static const char* k_states[] = { "NO_FIX", "FIX_PREDICT", "FIX_2D", "FIX_3D", "FIX_DIFF", "<FIX_ERROR>" };
		const char* navigation_state = k_states[std::min<uint8_t>(5, m->navigation_state)];
		LOG_ALWAYS("MESSAGE RECEIVER_STATE { TYPE: DF, PAYLOAD_LENGTH: %d, IOD: %d, NAV_STATE: %s, WEEK: %d, TOW: %f, X: %f, Y: %f, Z: %f, DX: %f, DY: %f, DZ: %f, CB: %f, CD: %f }", m->payload_length, m->iod, navigation_state, ntoh(m->wn), ntoh(m->tow), ntoh(m->ecef_pos_x), ntoh(m->ecef_pos_y), ntoh(m->ecef_pos_z), ntoh(m->ecef_vel_x), ntoh(m->ecef_vel_y), ntoh(m->ecef_vel_z), ntoh(m->clock_bias), ntoh(m->clock_drift));
	}
	break;
	case RLM3_GPS_MESSAGE_TYPE_E0_GPS_SUBFRAME: { auto m = (const RLM3_GPS_MESSAGE_E0_GPS_SUBFRAME*)message; LOG_ALWAYS("MESSAGE GPS_SUBFRAME { TYPE: E0, PAYLOAD_LENGTH: %d }", m->payload_length); } break;
	case RLM3_GPS_MESSAGE_TYPE_E1_GLONASS_STRING_BUFFER: { auto m = (const RLM3_GPS_MESSAGE_E1_GLONASS_STRING_BUFFER*)message; LOG_ALWAYS("MESSAGE GLONASS_STRING_BUFFER { TYPE: E1, PAYLOAD_LENGTH: %d }", m->payload_length); } break;
	case RLM3_GPS_MESSAGE_TYPE_E2_BEIDOU2_D1_SUBFRAME_BUFFER: { auto m = (const RLM3_GPS_MESSAGE_E2_BEIDOU2_D1_SUBFRAME_BUFFER*)message; LOG_ALWAYS("MESSAGE BEIDOU2_D1_SUBFRAME_BUFFER { TYPE: E2, PAYLOAD_LENGTH: %d }", m->payload_length); } break;
	case RLM3_GPS_MESSAGE_TYPE_E3_BEIDOU2_D2_SUBFRAME_BUFFER: { auto m = (const RLM3_GPS_MESSAGE_E3_BEIDOU2_D2_SUBFRAME_BUFFER*)message; LOG_ALWAYS("MESSAGE BEIDOU2_D2_SUBFRAME_BUFFER { TYPE: E3, PAYLOAD_LENGTH: %d }", m->payload_length); } break;
	default:
		LOG_ALWAYS("MESSAGE UNKNOWN { TYPE: %02x, PAYLOAD_LENGTH: %d }", message->message_type, message->payload_length);
		break;
	}
}

