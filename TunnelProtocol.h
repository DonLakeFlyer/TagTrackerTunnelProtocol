#pragma once

#include <cstdint>

namespace TunnelProtocol {

#define COMMAND_ID_ACK              1   // Ack response to command
#define COMMAND_ID_START_TAGS		2   // Previous tag set should be cleared, new tags are about to be uploaded
#define COMMAND_ID_END_TAGS			3   // All new tags have been uploaded
#define COMMAND_ID_TAG              4   // Tag info
#define COMMAND_ID_START_DETECTION  5   // Start pulse detection
#define COMMAND_ID_STOP_DETECTION   6   // Stop pulse detection
#define COMMAND_ID_PULSE           	7   // Detected pulse value
#define COMMAND_ID_RAW_CAPTURE      8 	// Capture raw sdr data
#define COMMAND_ID_HEARTBEAT	   	9  	// Heartbeat message
#define COMMAND_ID_START_ROTATION	10	// Start rotation, these ids are never sent as commands but are used to log the start and stop of rotation in the csv files
#define COMMAND_ID_STOP_ROTATION	11	// Cancel rotation, these ids are never sent as commands but are used to log the start and stop of rotation in the csv files
#define COMMAND_ID_SAVE_LOGS		12	// Save logs to usb sd card connected to rpi
#define COMMAND_ID_CLEAN_LOGS		13	// Clean logs from rpi

// AckInfo_t result values
#define COMMAND_RESULT_SUCCESS		1
#define COMMAND_RESULT_FAILURE		0

// HeartBeat_t codes
#define HEARTBEAT_SYSTEM_ID_MAVLINKCONTROLLER	1
#define HEARTBEAT_SYSTEM_ID_CHANNELIZER			2
#define HEARTBEAT_STATUS_IDLE					0	// Waiting for Tags to be sent
#define HEARTBEAT_STATUS_RECEIVING_TAGS			1	// In the middle fo tag receive sequence
#define HEARTBEAT_STATUS_HAS_TAGS				2	// Tags are known, waiting for detection start
#define HEARTBEAT_STATUS_DETECTING				3	// Detection is in progress
#define HEARTBEAT_STATUS_CAPTURE				4	// Capturing raw data

#ifndef MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN
#define MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN 128
#endif

typedef struct {
	uint32_t command;
} HeaderInfo_t;

typedef struct {
	HeaderInfo_t 	header;

	uint32_t		command;
	uint32_t		result;
    char            message[MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN - sizeof(HeaderInfo_t) - 2 * sizeof(uint32_t)];
} AckInfo_t;

typedef struct {
	HeaderInfo_t	header;

	// Tag id (uint 32)
	uint32_t		id;
	// Frequency (uint 32)
	uint32_t		frequency_hz;
	// Pulse duration
	uint32_t		pulse_width_msecs;
	// Intra-pulse duration 1
	uint32_t		intra_pulse1_msecs;
	// Intra-pulse duration 2
	uint32_t		intra_pulse2_msecs;
	// Intra-pulse uncertainty
	uint32_t		intra_pulse_uncertainty_msecs;
	// Intra-pulse jitter
	uint32_t		intra_pulse_jitter_msecs;
	// Number of pulses to integrate by
	uint32_t		k;
	// Probability of a false alarm
	double			false_alarm_probability;
	// The 1-based channel index from which this channel is output from the channelizer.
	uint32_t		channelizer_channel_number;
	// The center frequency of the above channel
	uint32_t		channelizer_channel_center_frequency_hz;
	// Pre-calculated threshold information for both pulse rates. NaN for not available
	double			ip1_mu;
	double			ip1_sigma;
	double			ip2_mu;
	double			ip2_sigma;
} TagInfo_t;

typedef struct {
    HeaderInfo_t	header;

	uint32_t		radio_center_frequency_hz;	// The center frequency to tune the radio to for sample collection
	uint32_t		gain;  						// Sensitivity gain setting for airspy 1- 21
} StartDetectionInfo_t;

typedef struct {
    HeaderInfo_t	header;
} StopDetectionInfo_t;

typedef struct {
    HeaderInfo_t	header;
} StartTagsInfo_t;

typedef struct {
    HeaderInfo_t	header;
} EndTagsInfo_t;

typedef struct {
    HeaderInfo_t	header;

	// Never sent as a command to the vehicle. Just documented here to specify the csv file format values
	double		latitude;
	double		longitude;
	double		altitude_AMSL;
} StartRotation_t;

typedef struct {
    HeaderInfo_t	header;

	// Never sent as a command to the vehicle. Just documented here to specify the csv file format values
	double		latitude;
	double		longitude;
	double		altitude_AMSL;
} StopRotation_t;

typedef struct {
    HeaderInfo_t	header;
} SaveLogsInfo_t;

typedef struct {
    HeaderInfo_t	header;
} CleanLogsInfo_t;

typedef struct {
    HeaderInfo_t	header;
	uint32_t		gain;  // Sensitivity gain setting for airspy 1- 21
} RawCaptureInfo_t;

typedef struct {
    HeaderInfo_t	header;

	// Descriptions and order are from the Interface Control Document
	// Tag ID (uint32_t)
	// The tag ID that was used for detection priori info. Useful for tractability.
	uint32_t 	tag_id;
	// Frequency (uint32_t)
	// Frequency at which pulse was detected. 0 value indicates detector heartbeat.
	uint32_t 	frequency_hz;
	// Time start (builtin_interfaces/Time (double))
	// System time at rising edge of pulse time bin.
	double 		start_time_seconds;
	// Expected next pulse time - start (builtin_interfaces/Time (double))
	// This is the time that the next pulse is expected to occur based on
	// the current pulse time and the priori pulse interval information.
	double 		predict_next_start_seconds;
	// Pulse SNR (float64/double)
	// Estimated pulse SNR in dB. This is the SNR during the time of pulse
	// transmission. Additionally, this is the ratio of the pulses peak
	// power point to the estimated noise power at that same frequency.
	// See ’Noise PSD’ parameter for how noise power is estimated.
	// SNR = 10 log10[(PSDS+N- PSDN )/PSDN].
	// Note: This value is currently being used as the signal strength metric.
	double 		snr;
	// STFT Score (float64/double)
	// TBD
	double 		stft_score;
	// For each pulse in a K group this value will be the same. This allows you
	// to be able to reconstruct the pulse group if needed. The value is incremented
	// with each new pulse group sent out over UDP/ROS.
	uint16_t 	group_seq_counter;
	// Pulse group index (uint16_t)
	// If more than one pulse is used for incoherent summing, the pulse group
	// will have up to K pulses. This property indicates where this pulse exists
	// in that pulse group. This property and the start time property can be
	// used to recollect pulse groups if needed.
	uint16_t 	group_ind;
	// Pulse group SNR (float64/double)
	// TBD
	double 		group_snr;
	// This is the estimated noise PSD at the frequency of the pulse.
	double		noise_psd;
	// Detection status (bool converted to uint8_t)
	// This property indicates if the pulse is a subthreshold pulse (0),
	// superthreshold pulse (1), or confirmed pulse (2). All confirmed pulses
	// are superthreshold pulses. Confirmed status mean that pulses in near
	// this frequency were previously detected and this pulse aligns in time
	// with predictions based on the prior pulse and the tag priori.
	uint8_t 	detection_status;
	// Confirmation status (bool converted to uint8_t)
	// This property indicates if the pulse has been confirmed (1), or is of yet
	// unconfirmed (0). Confirmed pulses had a preceding pulse that was detected
	// and projected a next pulse that aligned with this pulse.
	uint8_t 	confirmed_status;
	// This is the longitude of the antenna when the pulse was received. 
	double 		position_x;
	// This is the latitude of the antenna when the pulse was received. 
	double 		position_y;
	// This is the altitude of the antenna when the pulse was received in meters
	// above the launch location.
	double 		position_z;
	// x element of the antenna orientation quaternion in free space.
	float 		orientation_x;
	// y element of the antenna orientation quaternion in free space.
	float 		orientation_y;
	// z element of the antenna orientation quaternion in free space.
	float 		orientation_z;
	// w element of the antenna orientation quaternion in free space.
	float 		orientation_w;
} PulseInfo_t;

typedef struct {
    HeaderInfo_t	header;

	// Confirmation as to whether detection can be started or not
	uint32_t		status;
} StatusConfirmationInfo_t;

typedef struct {
	HeaderInfo_t 	header;

	uint16_t		system_id;
	uint16_t		status;
	float			cpu_temp_c;
} Heartbeat_t;

#define TunnelProtocolValidateSizes \
	((sizeof(TunnelProtocol::AckInfo_t) 				<= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN && \
	sizeof(TunnelProtocol::TagInfo_t) 					<= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN && \
	sizeof(TunnelProtocol::StartTagsInfo_t) 			<= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN && \
	sizeof(TunnelProtocol::EndTagsInfo_t) 				<= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN && \
	sizeof(TunnelProtocol::StartDetectionInfo_t) 		<= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN && \
	sizeof(TunnelProtocol::StopDetectionInfo_t) 		<= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN && \
	sizeof(TunnelProtocol::SaveLogsInfo_t) 			<= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN && \
	sizeof(TunnelProtocol::CleanLogsInfo_t) 		<= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN && \
	sizeof(TunnelProtocol::PulseInfo_t) 				<= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN && \
	sizeof(TunnelProtocol::RawCaptureInfo_t) 				<= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN && \
	sizeof(TunnelProtocol::Heartbeat_t) 				<= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN && \
	sizeof(TunnelProtocol::StatusConfirmationInfo_t) 	<= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN))

}
