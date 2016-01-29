// MESSAGE INT_SENSOR PACKING

#define MAVLINK_MSG_ID_INT_SENSOR 230

typedef struct __mavlink_int_sensor_t
{
 uint64_t time_usec; /*< Unix epoch time*/
 int32_t value; /*< measured value*/
 int32_t altitude; /*< Altitude*/
 int32_t longitude; /*< Longitude*/
 int32_t latitude; /*< Latitude*/
} mavlink_int_sensor_t;

#define MAVLINK_MSG_ID_INT_SENSOR_LEN 24
#define MAVLINK_MSG_ID_230_LEN 24

#define MAVLINK_MSG_ID_INT_SENSOR_CRC 103
#define MAVLINK_MSG_ID_230_CRC 103



#define MAVLINK_MESSAGE_INFO_INT_SENSOR { \
	"INT_SENSOR", \
	5, \
	{  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_int_sensor_t, time_usec) }, \
         { "value", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_int_sensor_t, value) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_int_sensor_t, altitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_int_sensor_t, longitude) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_int_sensor_t, latitude) }, \
         } \
}


/**
 * @brief Pack a int_sensor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param value measured value
 * @param time_usec Unix epoch time
 * @param altitude Altitude
 * @param longitude Longitude
 * @param latitude Latitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_int_sensor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int32_t value, uint64_t time_usec, int32_t altitude, int32_t longitude, int32_t latitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_INT_SENSOR_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, value);
	_mav_put_int32_t(buf, 12, altitude);
	_mav_put_int32_t(buf, 16, longitude);
	_mav_put_int32_t(buf, 20, latitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INT_SENSOR_LEN);
#else
	mavlink_int_sensor_t packet;
	packet.time_usec = time_usec;
	packet.value = value;
	packet.altitude = altitude;
	packet.longitude = longitude;
	packet.latitude = latitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INT_SENSOR_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_INT_SENSOR;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_INT_SENSOR_LEN, MAVLINK_MSG_ID_INT_SENSOR_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_INT_SENSOR_LEN);
#endif
}

/**
 * @brief Pack a int_sensor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param value measured value
 * @param time_usec Unix epoch time
 * @param altitude Altitude
 * @param longitude Longitude
 * @param latitude Latitude
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_int_sensor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int32_t value,uint64_t time_usec,int32_t altitude,int32_t longitude,int32_t latitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_INT_SENSOR_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, value);
	_mav_put_int32_t(buf, 12, altitude);
	_mav_put_int32_t(buf, 16, longitude);
	_mav_put_int32_t(buf, 20, latitude);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INT_SENSOR_LEN);
#else
	mavlink_int_sensor_t packet;
	packet.time_usec = time_usec;
	packet.value = value;
	packet.altitude = altitude;
	packet.longitude = longitude;
	packet.latitude = latitude;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INT_SENSOR_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_INT_SENSOR;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_INT_SENSOR_LEN, MAVLINK_MSG_ID_INT_SENSOR_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_INT_SENSOR_LEN);
#endif
}

/**
 * @brief Encode a int_sensor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param int_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_int_sensor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_int_sensor_t* int_sensor)
{
	return mavlink_msg_int_sensor_pack(system_id, component_id, msg, int_sensor->value, int_sensor->time_usec, int_sensor->altitude, int_sensor->longitude, int_sensor->latitude);
}

/**
 * @brief Encode a int_sensor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param int_sensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_int_sensor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_int_sensor_t* int_sensor)
{
	return mavlink_msg_int_sensor_pack_chan(system_id, component_id, chan, msg, int_sensor->value, int_sensor->time_usec, int_sensor->altitude, int_sensor->longitude, int_sensor->latitude);
}

/**
 * @brief Send a int_sensor message
 * @param chan MAVLink channel to send the message
 *
 * @param value measured value
 * @param time_usec Unix epoch time
 * @param altitude Altitude
 * @param longitude Longitude
 * @param latitude Latitude
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_int_sensor_send(mavlink_channel_t chan, int32_t value, uint64_t time_usec, int32_t altitude, int32_t longitude, int32_t latitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_INT_SENSOR_LEN];
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, value);
	_mav_put_int32_t(buf, 12, altitude);
	_mav_put_int32_t(buf, 16, longitude);
	_mav_put_int32_t(buf, 20, latitude);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INT_SENSOR, buf, MAVLINK_MSG_ID_INT_SENSOR_LEN, MAVLINK_MSG_ID_INT_SENSOR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INT_SENSOR, buf, MAVLINK_MSG_ID_INT_SENSOR_LEN);
#endif
#else
	mavlink_int_sensor_t packet;
	packet.time_usec = time_usec;
	packet.value = value;
	packet.altitude = altitude;
	packet.longitude = longitude;
	packet.latitude = latitude;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INT_SENSOR, (const char *)&packet, MAVLINK_MSG_ID_INT_SENSOR_LEN, MAVLINK_MSG_ID_INT_SENSOR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INT_SENSOR, (const char *)&packet, MAVLINK_MSG_ID_INT_SENSOR_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_INT_SENSOR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_int_sensor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t value, uint64_t time_usec, int32_t altitude, int32_t longitude, int32_t latitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, time_usec);
	_mav_put_int32_t(buf, 8, value);
	_mav_put_int32_t(buf, 12, altitude);
	_mav_put_int32_t(buf, 16, longitude);
	_mav_put_int32_t(buf, 20, latitude);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INT_SENSOR, buf, MAVLINK_MSG_ID_INT_SENSOR_LEN, MAVLINK_MSG_ID_INT_SENSOR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INT_SENSOR, buf, MAVLINK_MSG_ID_INT_SENSOR_LEN);
#endif
#else
	mavlink_int_sensor_t *packet = (mavlink_int_sensor_t *)msgbuf;
	packet->time_usec = time_usec;
	packet->value = value;
	packet->altitude = altitude;
	packet->longitude = longitude;
	packet->latitude = latitude;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INT_SENSOR, (const char *)packet, MAVLINK_MSG_ID_INT_SENSOR_LEN, MAVLINK_MSG_ID_INT_SENSOR_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INT_SENSOR, (const char *)packet, MAVLINK_MSG_ID_INT_SENSOR_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE INT_SENSOR UNPACKING


/**
 * @brief Get field value from int_sensor message
 *
 * @return measured value
 */
static inline int32_t mavlink_msg_int_sensor_get_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field time_usec from int_sensor message
 *
 * @return Unix epoch time
 */
static inline uint64_t mavlink_msg_int_sensor_get_time_usec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field altitude from int_sensor message
 *
 * @return Altitude
 */
static inline int32_t mavlink_msg_int_sensor_get_altitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field longitude from int_sensor message
 *
 * @return Longitude
 */
static inline int32_t mavlink_msg_int_sensor_get_longitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field latitude from int_sensor message
 *
 * @return Latitude
 */
static inline int32_t mavlink_msg_int_sensor_get_latitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Decode a int_sensor message into a struct
 *
 * @param msg The message to decode
 * @param int_sensor C-struct to decode the message contents into
 */
static inline void mavlink_msg_int_sensor_decode(const mavlink_message_t* msg, mavlink_int_sensor_t* int_sensor)
{
#if MAVLINK_NEED_BYTE_SWAP
	int_sensor->time_usec = mavlink_msg_int_sensor_get_time_usec(msg);
	int_sensor->value = mavlink_msg_int_sensor_get_value(msg);
	int_sensor->altitude = mavlink_msg_int_sensor_get_altitude(msg);
	int_sensor->longitude = mavlink_msg_int_sensor_get_longitude(msg);
	int_sensor->latitude = mavlink_msg_int_sensor_get_latitude(msg);
#else
	memcpy(int_sensor, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_INT_SENSOR_LEN);
#endif
}
