#pragma once

#include <cstdint>

namespace TunnelProtocol {

// Increment for every incompatible wire-layout or command-semantics change.
#define TUNNEL_PROTOCOL_VERSION 1

#define COMMAND_ID_ACK              			1   // Ack response to command
#define COMMAND_ID_START_TAGS					2   // Previous tag set should be cleared, new tags are about to be uploaded
#define COMMAND_ID_END_TAGS						3   // All new tags have been uploaded
#define COMMAND_ID_TAG              			4   // Tag info
#define COMMAND_ID_START_DETECTION  			5   // Start pulse detection
#define COMMAND_ID_STOP_DETECTION  				6   // Stop pulse detection
#define COMMAND_ID_PULSE           				7   // Detected pulse value
#define COMMAND_ID_RAW_CAPTURE      			8 	// Capture raw sdr data
#define COMMAND_ID_HEARTBEAT	   				9  	// Heartbeat message
#define COMMAND_ID_START_ROTATION				10	// Start rotation, these ids are never sent as commands but are used to log the start and stop of rotation in the csv files
#define COMMAND_ID_STOP_ROTATION				11	// Cancel rotation, these ids are never sent as commands but are used to log the start and stop of rotation in the csv files
#define COMMAND_ID_SAVE_LOGS					12	// Save logs to usb sd card connected to rpi
#define COMMAND_ID_CLEAN_LOGS					13	// Clean logs from rpi
#define COMMAND_ID_AIRSPY_STATUS				14	// Query whether an AirSpy device is connected
#define COMMAND_ID_START_COLLECTION             15  // Start persistent Python detector collection
#define COMMAND_ID_START_COLLECTION_SLICE       16  // Arm one heading slice on the running collection
#define COMMAND_ID_FINISH_COLLECTION            17  // Finalize or cancel a persistent collection
#define COMMAND_ID_BEARING_RESULT				18	// Bearing calculation result sent to GCS (Python detector only)
#define COMMAND_ID_COLLECTION_STATUS            19  // Asynchronous collection lifecycle event

#define COLLECTION_FINISH_FINALIZE  0
#define COLLECTION_FINISH_CANCEL    1

#define COLLECTION_STATUS_SLICE_ARMED     1
#define COLLECTION_STATUS_SLICE_COMPLETE  2
#define COLLECTION_STATUS_FAILED          3
#define COLLECTION_STATUS_STOPPED         4

// AckInfo_t result values
#define COMMAND_RESULT_SUCCESS		1
#define COMMAND_RESULT_FAILURE		0

// Detection mode for StartDetectionInfo_t
#define DETECTION_MODE_UAVRT		0	// uavrt_detection C++ detector (default)
#define DETECTION_MODE_PYTHON		1	// Python pulse_detector.py

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
	// Intra-pulse duration 2airspy_info
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
	uint32_t		detection_mode;				// DETECTION_MODE_UAVRT (0) or DETECTION_MODE_PYTHON (1)

	// Python detector only: detection sensitivity and confidence thresholds.
	// detection_margin: multiplier on EVT threshold (e.g. 0.90 = 10% more sensitive). 0 = use default (0.90).
	// confidence_ratio: score/threshold ratio for confirmed status (e.g. 1.3). 0 = use default (1.3).
	// These fields are ignored by uavrt_detection (C++ detector).
	double			detection_margin;
	double			confidence_ratio;

	// Enable verbose per-hypothesis debug logging in the Python detector.
	// 0 = normal logging, non-zero = enable --debug flag.
	// Also enables debug output for uavrt_detection when non-zero.
	uint32_t		debug_detector;

	// Enable per-cycle spectrogram and IQ dump to the log directory.
	// 0 = disabled, non-zero = save power.npy + iq.npy + meta.json per cycle.
	uint32_t		dump_spectrogram;
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
	uint32_t		gain;  			// Sensitivity gain setting for airspy 1- 21
	uint32_t		frequency_hz;	// Frequency to tune to for capture
} RawCaptureInfo_t;

typedef struct {
    HeaderInfo_t	header;
} AirspyStatusInfo_t;

typedef struct {
    HeaderInfo_t	header;

    uint32_t        collection_id;
	uint32_t		radio_center_frequency_hz;	// Center frequency for SDR tuning
	uint32_t		n_slices;					// Number of heading slices (informational)
	double			detection_margin;			// EVT threshold multiplier (0 = use default 0.90)
	double			confidence_ratio;			// Score/threshold ratio for confirmed status (0 = use default 1.3)
	uint32_t		debug_detector;				// Enable verbose debug logging (0 = off, non-zero = on)
	uint32_t		dump_spectrogram;			// Enable spectrogram/IQ dump (0 = off, non-zero = on)
} StartCollection_t;

typedef struct {
    HeaderInfo_t	header;

    uint32_t        collection_id;
    uint32_t        slice_id;
	float			heading_deg;				// Aircraft heading in degrees for this detection slice
} StartCollectionSlice_t;

typedef struct {
    HeaderInfo_t	header;
    uint32_t        collection_id;
    uint32_t        disposition;             // COLLECTION_FINISH_FINALIZE or COLLECTION_FINISH_CANCEL
} FinishCollection_t;

typedef struct {
    HeaderInfo_t    header;
    uint32_t        collection_id;
    uint32_t        slice_id;
    uint32_t        status;
    uint32_t        expected_detectors;
    uint32_t        completed_detectors;
    uint32_t        error_code;
} CollectionStatus_t;

typedef struct {
    HeaderInfo_t	header;

    uint32_t        collection_id;
	uint32_t		tag_id;						// Tag ID for this bearing result
	float			bearing_deg;				// Estimated bearing to transmitter (degrees)
	float			r_squared;					// Goodness of fit (0..1)
	uint32_t		n_valid_slices;				// Number of heading slices with confirmed detections
	float			best_snr;					// Best SNR observed across all slices (dB)
} BearingResult_t;

typedef struct {
    HeaderInfo_t	header;
    uint32_t    collection_id;
    uint32_t    slice_id;

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
	// Pulse group index / rate-switch hypothesis (uint16_t)
	//
	// C++ detector (DETECTION_MODE_UAVRT):
	//   Index of this pulse within its K-pulse group (0..K-1).
	//   Secondary-rate detections arrive on a separate tag_id (id + 1).
	//
	// Python detector (DETECTION_MODE_PYTHON):
	//   Encodes the winning rate-switch hypothesis for the K-group.
	//   Both rates are handled in a single detector process (same tag_id).
	//     0          = pure rate A (primary/resting TIP)
	//     1          = pure rate B (secondary/moving TIP)
	//     2..K-1     = A→B switch at change-point c  (group_ind = 1 + c)
	//     K..2K-3    = B→A switch at change-point c  (group_ind = K - 1 + c)
	//   When intra_pulse2_msecs is 0 (single-rate tag), group_ind is
	//   always 0.
	uint16_t 	group_ind;
	// Python detector: fixed-offset absolute signal power for the K-pulse
	// group, computed as sum(power) - K*noise with no local-max pooling.
	// This value is intentionally not clamped and may be negative.
	// C++ detector: legacy pulse group SNR.
	double 		group_snr;
	// This is the estimated noise PSD at the frequency of the pulse.
	double		noise_psd;
	// Detection status (uint8_t)
	//   0 = subthreshold pulse
	//   1 = superthreshold pulse
	//   2 = confirmed pulse (superthreshold + aligned with prior prediction)
	//   3 = no pulse detected (detector searched this cycle and found nothing;
	//       noise_psd carries the observed noise floor).
	//       C++ detector: two separate processes for dual-rate tags, so two
	//         no-detection messages per cycle (one per tag_id).
	//       Python detector: single process handles both rates, so only one
	//         no-detection message per cycle per tag.
	//       For status 3: snr=0, predict_next=0, group_ind=0,
	//       stft_score carries the best sub-threshold score ratio
	//       (Python) or 0 (C++).
	uint8_t 	detection_status;
	// Confirmation status (bool converted to uint8_t)
	// This property indicates if the pulse has been confirmed (1), or is of yet
	// unconfirmed (0). Confirmed pulses had a preceding pulse that was detected
	// and projected a next pulse that aligned with this pulse.
	uint8_t 	confirmed_status;
	// Latitude of the antenna when the pulse was received (degrees).
	double 		latitude;
	// Longitude of the antenna when the pulse was received (degrees).
	double 		longitude;
	// Altitude of the antenna when the pulse was received (meters above launch).
	double 		altitude_rel;
	// Roll angle of the antenna in degrees.
	float 		roll_deg;
	// Pitch angle of the antenna in degrees.
	float 		pitch_deg;
	// Yaw (heading) angle of the antenna in degrees.
	float 		yaw_deg;
} PulseInfo_t;

static constexpr uint8_t kSubthresholdDetectionStatus   = 0;
static constexpr uint8_t kSuperthresholdDetectionStatus  = 1;
static constexpr uint8_t kConfirmedDetectionStatus       = 2;
static constexpr uint8_t kNoPulseDetectionStatus         = 3;

typedef struct {
    HeaderInfo_t	header;

	// Confirmation as to whether detection can be started or not
	uint32_t		status;
} StatusConfirmationInfo_t;

typedef struct {
	HeaderInfo_t 	header;

    uint32_t        protocol_version;
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
	sizeof(TunnelProtocol::AirspyStatusInfo_t) 				<= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN && \
	sizeof(TunnelProtocol::Heartbeat_t) 				<= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN && \
	sizeof(TunnelProtocol::StatusConfirmationInfo_t) 	<= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN && \
    sizeof(TunnelProtocol::StartCollection_t)             <= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN && \
    sizeof(TunnelProtocol::StartCollectionSlice_t)        <= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN && \
    sizeof(TunnelProtocol::FinishCollection_t)            <= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN && \
    sizeof(TunnelProtocol::CollectionStatus_t)            <= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN && \
	sizeof(TunnelProtocol::BearingResult_t) 			<= MAVLINK_MSG_TUNNEL_FIELD_PAYLOAD_LEN))

}
