// MESSAGE SENSOR PACKING

#define MAVLINK_MSG_ID_SENSOR 200

typedef struct __mavlink_sensor_t
{
 uint64_t time_usec; ///< Unix epoch time
 int32_t value; ///< measured value
 int32_t altitude; ///< Altitude
 int32_t longitude; ///< Longitude
 int32_t latitude; ///< Latitude
 char sensorType[16]; ///< Type of the sensor
} mavlink_sensor_t;

#define MAVLINK_MSG_ID_SENSOR_LEN 40
#define MAVLINK_MSG_ID_200_LEN 40

#define MAVLINK_MSG_ID_SENSOR_CRC 6
#define MAVLINK_MSG_ID_200_CRC 6

#define MAVLINK_MSG_SENSOR_FIELD_SENSORTYPE_LEN 16

#define MAVLINK_MESSAGE_INFO_SENSOR { \
	"SENSOR", \
	6, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sensor_t, time_usec) }, \
         { "value", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_sensor_t, value) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_sensor_t, altitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_sensor_t, longitude) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_sensor_t, latitude) }, \
         { "sensorType", NULL, MAVLINK_TYPE_CHAR, 16, 24, offsetof(mavlink_sensor_t, sensorType) }, \
         } \
}


/**
 * @brief Pack a sensor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param value measured value
 * @param sensorType Type of the sensor
 * @param time_usec Unix epoch time
 * @param altitude Altitude
 * @param longitude Longitude
 * @param latitude Latitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int32_t value, const char *sensorType, uint64_t time_usec, int32_t altitude, int32_t longitude, int32_t latitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SENSOR_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, value);
	_mav_put_int32_t(buf, 12, altitude);
	_mav_put_int32_t(buf, 16, longitude);
	_mav_put_int32_t(buf, 20, latitude);
	_mav_put_char_array(buf, 24, sensorType, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_LEN);
#else
	mavlink_sensor_t packet;
	packet.time_usec = time_usec;
	packet.value = value;
	packet.altitude = altitude;
	packet.longitude = longitude;
	packet.latitude = latitude;
	mav_array_memcpy(packet.sensorType, sensorType, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SENSOR;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENSOR_LEN, MAVLINK_MSG_ID_SENSOR_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENSOR_LEN);
#endif
}

/**
 * @brief Pack a sensor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param value measured value
 * @param sensorType Type of the sensor
 * @param time_usec Unix epoch time
 * @param altitude Altitude
 * @param longitude Longitude
 * @param latitude Latitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sensor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int32_t value,const char *sensorType,uint64_t time_usec,int32_t altitude,int32_t longitude,int32_t latitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SENSOR_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, value);
	_mav_put_int32_t(buf, 12, altitude);
	_mav_put_int32_t(buf, 16, longitude);
	_mav_put_int32_t(buf, 20, latitude);
	_mav_put_char_array(buf, 24, sensorType, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENSOR_LEN);
#else
	mavlink_sensor_t packet;
	packet.time_usec = time_usec;
	packet.value = value;
	packet.altitude = altitude;
	packet.longitude = longitude;
	packet.latitude = latitude;
	mav_array_memcpy(packet.sensorType, sensorType, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENSOR_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SENSOR;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENSOR_LEN, MAVLINK_MSG_ID_SENSOR_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENSOR_LEN);
#endif
}

/**
 * @brief Encode a sensor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sensor_t* sensor)
{
	return mavlink_msg_sensor_pack(system_id, component_id, msg, sensor->value, sensor->sensorType, sensor->time_usec, sensor->altitude, sensor->longitude, sensor->latitude);
}

/**
 * @brief Encode a sensor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sensor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sensor_t* sensor)
{
	return mavlink_msg_sensor_pack_chan(system_id, component_id, chan, msg, sensor->value, sensor->sensorType, sensor->time_usec, sensor->altitude, sensor->longitude, sensor->latitude);
}

/**
 * @brief Send a sensor message
 * @param chan MAVLink channel to send the message
 *
 * @param value measured value
 * @param sensorType Type of the sensor
 * @param time_usec Unix epoch time
 * @param altitude Altitude
 * @param longitude Longitude
 * @param latitude Latitude
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sensor_send(mavlink_channel_t chan, int32_t value, const char *sensorType, uint64_t time_usec, int32_t altitude, int32_t longitude, int32_t latitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SENSOR_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, value);
	_mav_put_int32_t(buf, 12, altitude);
	_mav_put_int32_t(buf, 16, longitude);
	_mav_put_int32_t(buf, 20, latitude);
	_mav_put_char_array(buf, 24, sensorType, 16);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR, buf, MAVLINK_MSG_ID_SENSOR_LEN, MAVLINK_MSG_ID_SENSOR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR, buf, MAVLINK_MSG_ID_SENSOR_LEN);
#endif
#else
	mavlink_sensor_t packet;
	packet.time_usec = time_usec;
	packet.value = value;
	packet.altitude = altitude;
	packet.longitude = longitude;
	packet.latitude = latitude;
	mav_array_memcpy(packet.sensorType, sensorType, sizeof(char)*16);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR, (const char *)&packet, MAVLINK_MSG_ID_SENSOR_LEN, MAVLINK_MSG_ID_SENSOR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR, (const char *)&packet, MAVLINK_MSG_ID_SENSOR_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SENSOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sensor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t value, const char *sensorType, uint64_t time_usec, int32_t altitude, int32_t longitude, int32_t latitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, value);
	_mav_put_int32_t(buf, 12, altitude);
	_mav_put_int32_t(buf, 16, longitude);
	_mav_put_int32_t(buf, 20, latitude);
	_mav_put_char_array(buf, 24, sensorType, 16);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR, buf, MAVLINK_MSG_ID_SENSOR_LEN, MAVLINK_MSG_ID_SENSOR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR, buf, MAVLINK_MSG_ID_SENSOR_LEN);
#endif
#else
	mavlink_sensor_t *packet = (mavlink_sensor_t *)msgbuf;
	packet->time_usec = time_usec;
	packet->value = value;
	packet->altitude = altitude;
	packet->longitude = longitude;
	packet->latitude = latitude;
	mav_array_memcpy(packet->sensorType, sensorType, sizeof(char)*16);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR, (const char *)packet, MAVLINK_MSG_ID_SENSOR_LEN, MAVLINK_MSG_ID_SENSOR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENSOR, (const char *)packet, MAVLINK_MSG_ID_SENSOR_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SENSOR UNPACKING


/**
 * @brief Get field value from sensor message
 *
 * @return measured value
 */
static inline int32_t mavlink_msg_sensor_get_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field sensorType from sensor message
 *
 * @return Type of the sensor
 */
static inline uint16_t mavlink_msg_sensor_get_sensorType(const mavlink_message_t* msg, char *sensorType)
{
	return _MAV_RETURN_char_array(msg, sensorType, 16,  24);
}

/**
 * @brief Get field time_usec from sensor message
 *
 * @return Unix epoch time
 */
static inline uint64_t mavlink_msg_sensor_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field altitude from sensor message
 *
 * @return Altitude
 */
static inline int32_t mavlink_msg_sensor_get_altitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field longitude from sensor message
 *
 * @return Longitude
 */
static inline int32_t mavlink_msg_sensor_get_longitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field latitude from sensor message
 *
 * @return Latitude
 */
static inline int32_t mavlink_msg_sensor_get_latitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Decode a sensor message into a struct
 *
 * @param msg The message to decode
 * @param sensor C-struct to decode the message contents into
 */
static inline void mavlink_msg_sensor_decode(const mavlink_message_t* msg, mavlink_sensor_t* sensor)
{
#if MAVLINK_NEED_BYTE_SWAP
	sensor->time_usec = mavlink_msg_sensor_get_time_usec(msg);
	sensor->value = mavlink_msg_sensor_get_value(msg);
	sensor->altitude = mavlink_msg_sensor_get_altitude(msg);
	sensor->longitude = mavlink_msg_sensor_get_longitude(msg);
	sensor->latitude = mavlink_msg_sensor_get_latitude(msg);
	mavlink_msg_sensor_get_sensorType(msg, sensor->sensorType);
#else
	memcpy(sensor, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SENSOR_LEN);
#endif
}
