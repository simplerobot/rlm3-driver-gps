#pragma once

#include "rlm3-base.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef enum RLM3_GPS_Error
{
	RLM3_GPS_ERROR_INTERNAL,
	RLM3_GPS_ERROR_BUFFER_FULL,
	RLM3_GPS_ERROR_CHECKSUM_FAIL,
	RLM3_GPS_ERROR_CHANNEL_FAIL,
	RLM3_GPS_ERROR_PROTOCOL_FAIL,
} RLM3_GPS_Error;


typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t data[0];
} RLM3_GPS_MESSAGE;


#define RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(MSG) (sizeof(MSG) - sizeof((MSG).payload_length))
#define RLM3_GPS_SET_MESSAGE_PAYLOAD_SIZE(MSG) ((MSG).payload_length = RLM3_GPS_GET_MESSAGE_PAYLOAD_SIZE(MSG))


extern void RLM3_GPS_Init();
extern void RLM3_GPS_Deinit();
extern bool RLM3_GPS_IsInit();

extern const RLM3_GPS_MESSAGE* RLM3_GPS_GetNextMessage(size_t timeout_ms);
extern bool RLM3_GPS_SendMessage(const RLM3_GPS_MESSAGE* message);

extern void RLM3_GPS_PulseCallback();
extern void RLM3_GPS_ErrorCallback(RLM3_GPS_Error error);




uint16_t GPS_To_Host_U16(uint16_t value);


static const uint8_t RLM3_GPS_MESSAGE_TYPE_01_SYSTEM_RESTART                                = 0x01;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_02_QUERY_SOFTWARE_VERSION                        = 0x02;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_03_QUERY_SOFTWARE_CRC                            = 0x03;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_04_SET_FACTORY_DEFAULTS                          = 0x04;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_05_CONFIGURE_SERIAL_PORT                         = 0x05;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_08_CONFIGURE_NMEA_MESSAGE                        = 0x08;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_09_CONFIGURE_OUTPUT_MESSAGE_FORMAT               = 0x09;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_0E_CONFIGURE_SYSTEM_POSITION_RATE                = 0x0E;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_10_QUERY_SYSTEM_POSITION_UPDATE_RATE             = 0x10;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_1E_CONFIGURE_BINARY_MEASUREMENT_DATA_OUTPUT      = 0x1E;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_1F_QUERY_BINARY_MEASUREMENT_DATA_OUTPUT_STATUS   = 0x1F;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_30_GET_GPS_EPHEMERIS                             = 0x30;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_31_SET_EPHEMERIS                                 = 0x31;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_5B_GET_GLONASS_EPHEMERIS                         = 0x5B;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_5C_SET_GLONASS_EPHEMERIS                         = 0x5C;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_80_SOFTWARE_VERSION                              = 0x80;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_81_SOFTWARE_CRC                                  = 0x81;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_83_ACK                                           = 0x83;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_84_NACK                                          = 0x84;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_86_POSITION_UPDATE_RATE                          = 0x86;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_89_BINARY_MEASUREMENT_DATA_OUTPUT_STATUS         = 0x89;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_90_GLONASS_EPHEMERIS_DATA                        = 0x90;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_B1_GPS_EPHEMERIS_DATA                            = 0xB1;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_DC_MEASUREMENT_TIME                              = 0xDC;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_DD_RAW_MEASUREMENT                               = 0xDD;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_DE_SV_CH_STATUS                                  = 0xDE;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_DF_RECEIVER_STATE                                = 0xDF;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_E0_GPS_SUBFRAME                                  = 0xE0;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_E1_GLONASS_STRING_BUFFER                         = 0xE1;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_E2_BEIDOU2_D1_SUBFRAME_BUFFER                    = 0xE2;
static const uint8_t RLM3_GPS_MESSAGE_TYPE_E3_BEIDOU2_D2_SUBFRAME_BUFFER                    = 0xE3;


typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t start_mode; // 0 = No Change, 1 = Hot Start, 2 = Warm Start, 3 = Cold Start, 4 = Test Mode.
	uint16_t utc_year; // >= 1980
	uint8_t utc_month; // 1 - 12
	uint8_t utc_day; // 1 - 31
	uint8_t utc_hour; // 0 - 23
	uint8_t utc_minute; // 0 - 59
	uint8_t utc_second; // 0 - 59
	int16_t latitude; // -9000 - 9000 (1/100 degree) Positive values are Northern Hemisphere
	int16_t longitude; // -18000 - 18000 (1 /100 degree) Positive values are Eastern Hemisphere.
	int16_t altitude; // -1000 - 18300 (meters)
} RLM3_GPS_MESSAGE_01_SYSTEM_RESTART;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t software_type; // 0: Reserved 1: System code
} RLM3_GPS_MESSAGE_02_QUERY_SOFTWARE_VERSION;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t software_type; // 0: Reserved 1: System code
} RLM3_GPS_MESSAGE_03_QUERY_SOFTWARE_CRC;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t type; // 0: Reserved 1: Reboot
} RLM3_GPS_MESSAGE_04_SET_FACTORY_DEFAULTS;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t com_port; // 0 = COM 1
	uint8_t baud_rate; // 0 = 4800, 1 = 9600, 2 = 19200, 3 = 38400, 4 = 57600, 5 = 115200, 6 = 230400, 7 = 460800, 8 = 921600
	uint8_t attributes; // 0 = Update to SRAM, 1 = Update to SRAM and FLASH
} RLM3_GPS_MESSAGE_05_CONFIGURE_SERIAL_PORT;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t gga_interval; // 0 = disable, 1 - 255 = seconds
	uint8_t gsa_interval; // 0 = disable, 1 - 255 = seconds
	uint8_t gsv_interval; // 0 = disable, 1 - 255 = seconds
	uint8_t gll_interval; // 0 = disable, 1 - 255 = seconds
	uint8_t rmc_interval; // 0 = disable, 1 - 255 = seconds
	uint8_t vtg_interval; // 0 = disable, 1 - 255 = seconds
	uint8_t zda_interval; // 0 = disable, 1 - 255 = seconds
	uint8_t attributes; // 0 = Update to SRAM, 1 = Update to SRAM and FLASH
} RLM3_GPS_MESSAGE_08_CONFIGURE_NMEA_MESSAGE;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t type; // 0 = No output, 1 = NMEA message, 2 = Binary message
	uint8_t attributes; // 0 = Update to SRAM, 1 = Update to SRAM and FLASH
} RLM3_GPS_MESSAGE_09_CONFIGURE_OUTPUT_MESSAGE_FORMAT;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t rate; // 1, 2, 4, 5, 8, 10, 20, 25, 40, 50 (Hz) Values over 20 need faster baud rate.
	uint8_t attributes; // 0 = Update to SRAM, 1 = Update to SRAM and FLASH
} RLM3_GPS_MESSAGE_0E_CONFIGURE_SYSTEM_POSITION_RATE;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
} RLM3_GPS_MESSAGE_10_QUERY_SYSTEM_POSITION_UPDATE_RATE;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t output_rate; // 0 = 1Hz, 1 = 2Hz, 2 = 4Hz, 3 = 5Hz, 4 = 10Hz, 5 = 20Hz
	uint8_t meas_time_enabling; // 0 = Disable, 1 = Enable
	uint8_t raw_meas_enabling; // 0 = Disable, 1 = Enable
	uint8_t sv_ch_status_enabling; // 0 = Disable, 1 = Enable
	uint8_t rcv_state_enabling; // 0 = Disable, 1 = Enable
	uint8_t subframe_enabling; // Bit 0 = GPS, Bit 1 = GLONASS, Bit 2 = Galileo, Bit 3 = Beidou
	uint8_t attributes; // 0 = Update to SRAM, 1 = Update to SRAM and FLASH
	uint8_t reserved; // What is this?  Must be zero?  Is this attributes? This does not match the spec.
} RLM3_GPS_MESSAGE_1E_CONFIGURE_BINARY_MEASUREMENT_DATA_OUTPUT;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
} RLM3_GPS_MESSAGE_1F_QUERY_BINARY_MEASUREMENT_DATA_OUTPUT_STATUS;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t sv_id; // satellite id: 0 = All SVs, 1-32 = Particular SV.
} RLM3_GPS_MESSAGE_30_GET_GPS_EPHEMERIS;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint16_t sv_id; // satellite id: 1-32
	uint8_t subframe_data[3][28]; // Ephemeris subframe data.
} RLM3_GPS_MESSAGE_31_SET_EPHEMERIS;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t sv_id; // satellite id: 0 = All SVs, 1-32 = Particular SV.
} RLM3_GPS_MESSAGE_5B_GET_GLONASS_EPHEMERIS;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t slot_number; // GLONASS SV slot number
	uint8_t k_number; // GLONASS SV Frequency number: -7 - 6
	uint8_t glo_eph_data[4][10]; // Ephemeris data.
} RLM3_GPS_MESSAGE_5C_SET_GLONASS_EPHEMERIS;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t software_type; // 0: Reserved 1: System code
	uint32_t kernel_version;
	uint32_t odm_version;
	uint32_t revision;
} RLM3_GPS_MESSAGE_80_SOFTWARE_VERSION;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t software_type; // 0: Reserved 1: System code
	uint16_t crc;
} RLM3_GPS_MESSAGE_81_SOFTWARE_CRC;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t ack_id; // Message ID of the request message
} RLM3_GPS_MESSAGE_83_ACK;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t nack_id;// Message ID of the request message
} RLM3_GPS_MESSAGE_84_NACK;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t update_rate; // Position update rate of the GNSS system (Hz)
} RLM3_GPS_MESSAGE_86_POSITION_UPDATE_RATE;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t output_rate; // 0 = 1Hz, 1 = 2Hz, 2 = 4Hz, 3 = 5Hz, 4 = 10Hz, 5 = 20Hz
	uint8_t meas_time_enabling; // 0 = Disable, 1 = Enable
	uint8_t raw_meas_enabling; // 0 = Disable, 1 = Enable
	uint8_t sv_ch_status_enabling; // 0 = Disable, 1 = Enable
	uint8_t rcv_state_enabling; // 0 = Disable, 1 = Enable
	uint8_t subframe_enabling; // Bit 0 = GPS, Bit 1 = GLONASS, Bit 2 = Galileo, Bit 3 = Beidou
} RLM3_GPS_MESSAGE_89_BINARY_MEASUREMENT_DATA_OUTPUT_STATUS;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t slot_number; // GLONASS SV slot number
	uint8_t k_number; // GLONASS SV Frequency number: -7 - 6
	uint8_t glo_eph_data[4][10]; // Ephemeris data.
} RLM3_GPS_MESSAGE_90_GLONASS_EPHEMERIS_DATA;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint16_t sv_id; // satellite id: 1-32
	uint8_t subframe_data[3][28]; // Ephemeris subframe data.
} RLM3_GPS_MESSAGE_B1_GPS_EPHEMERIS_DATA;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t iod;
	uint16_t receiver_wn; // Receiver Week Number (weeks)
	uint32_t receiver_tow; // Receiver Time Of Week (ms)
	uint16_t measurement_period; // (ms)
} RLM3_GPS_MESSAGE_DC_MEASUREMENT_TIME;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t iod; // Issue of data
	uint8_t nmeas; // Number of measurements
	struct __attribute__((__packed__))
	{
		uint8_t svid; // PRN for GPS satellites; Slot + 64 for GLONASS satellites; Slot + 200 for Beidou2 satellites
		uint8_t cn0; // Satellite CNR (dBHz)
		double pseudo_range; // Satellite pseudo-range (meters)
		double accumulated_carrier_cycles; // Accumulated carrier phase measurement (L1 Cycles)
		float doppler_frequency; // Satellite doppler frequency approaching satellite has positive frequency. (Hz)
		uint8_t measurement_indicator; // Bit 0 = pseudo-range available, Bit 1 = Doppler available, Bit 2 = Carrier phase available, Bit 3 = cycle slip possible, Bit 4 = coherent integration time exceeds 10ms
	} channels[1];
} RLM3_GPS_MESSAGE_DD_RAW_MEASUREMENT;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t iod; // Issue of data
	uint8_t nsvs; // Number of SVs
	struct __attribute__((__packed__))
	{
		uint8_t channel_id;
		uint8_t svid; // PRN for GPS satellites; Slot + 64 for GLONASS satellites; Slot + 200 for Beidou2 satellites
		uint8_t sv_status_indicator; // Bit 0 = Almanac received, Bit 1 = Ephemeris received, Bit 2 = Satellite Healthy
		uint8_t ura; // URA index for GPS satellites, Ft parameter for GLONASS satellites.  255 indicates value is not available.
		uint8_t cn0; // Satellite CNR (dBHz)
		int16_t elevation; // SV Elevation (degrees)
		int16_t azimoth; // SV Azimuth (degrees)
		uint8_t channel_status_indicator; // Bit 0 = Pull-in stage done, Bit 1 = synchronization done, Bit 2 = frame synchronization done, Bit 3 = Ephemeris received, Bit 4 = Used in normal fix, Bit 5 = Used in differential fix mode
	} channels[1];
} RLM3_GPS_MESSAGE_DE_SV_CH_STATUS;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t iod; // Issue of data
	uint8_t navigation_state; // 0 = No Fix, 1 = Fix Prediction, 2 = Fix 2D, 3 = Fix 3D, 4 = Fix Differential
	uint16_t wn; // GPS week number (weeks)
	double tow; // GPS Time of Week (seconds)
	double ecef_pos_x; // (meters)
	double ecef_pos_y; // (meters)
	double ecef_pos_z; // (meters)
	float ecef_vel_x; // (m/s)
	float ecef_vel_y; // (m/s)
	float ecef_vel_z; // (m/s)
	double clock_bias; // (meters)
	float clock_drift; // (m/s)
	float gdop; // geometric dilution of precision
	float pdop; // position dilution of precision
	float hdop; // horizontal dilution of precision
	float vdop; // vertical dilution of precision
	float tdop; // total dilution of precision
} RLM3_GPS_MESSAGE_DF_RECEIVER_STATE;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t svid; // GPS Satellite PRN
	uint8_t sfid; // Sub-frame ID
	uint8_t subframe[30]; // Subframe data.
} RLM3_GPS_MESSAGE_E0_GPS_SUBFRAME;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t svid; // GLONASS satellite slot number + 64
	uint8_t string_number; // String number of navigation message
	uint8_t data[9]; // Bit Data.
} RLM3_GPS_MESSAGE_E1_GLONASS_STRING_BUFFER;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t svid; // BEIDOU2 D1 Satellite SVID + 200
	uint8_t sfid; // Sub-frame ID
	uint8_t data[28]; // Subframe data.
} RLM3_GPS_MESSAGE_E2_BEIDOU2_D1_SUBFRAME_BUFFER;

typedef struct __attribute__((__packed__))
{
	uint16_t payload_length;
	uint8_t message_type;
	uint8_t svid; // BEIDOU2 D1 Satellite SVID + 200
	uint8_t sfid; // Sub-frame ID
	uint8_t data[28]; // Subframe data.
} RLM3_GPS_MESSAGE_E3_BEIDOU2_D2_SUBFRAME_BUFFER;


#ifdef __cplusplus
}
#endif
