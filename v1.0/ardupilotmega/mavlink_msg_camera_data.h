// MESSAGE CAMERA_DATA PACKING

#define MAVLINK_MSG_ID_CAMERA_DATA 180

typedef struct __mavlink_camera_data_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 int32_t time_week; ///< GPS week number
 uint32_t time_week_ms; ///< GPS time (milliseconds from start of GPS week)
 float roll; ///< Roll angle (rad, -pi..+pi)
 float pitch; ///< Pitch angle (rad, -pi..+pi)
 float yaw; ///< Yaw angle (rad, -pi..+pi)
 int32_t lat; ///< Latitude, expressed as * 1E7
 int32_t lon; ///< Longitude, expressed as * 1E7
 int32_t alt; ///< Altitude in meters, expressed as * 1000 (millimeters), above MSL
 int32_t relative_alt; ///< Altitude above ground in meters, expressed as * 1000 (millimeters)
 uint16_t hdg; ///< Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 uint8_t fix_type; ///< 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 uint8_t satellites_visible; ///< Number of satellites visible. If unknown, set to 255
} mavlink_camera_data_t;

#define MAVLINK_MSG_ID_CAMERA_DATA_LEN 44
#define MAVLINK_MSG_ID_180_LEN 44

#define MAVLINK_MSG_ID_CAMERA_DATA_CRC 45
#define MAVLINK_MSG_ID_180_CRC 45



#define MAVLINK_MESSAGE_INFO_CAMERA_DATA { \
	"CAMERA_DATA", \
	13, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_camera_data_t, time_boot_ms) }, \
         { "time_week", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_camera_data_t, time_week) }, \
         { "time_week_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_camera_data_t, time_week_ms) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_camera_data_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_camera_data_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_camera_data_t, yaw) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_camera_data_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_camera_data_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_camera_data_t, alt) }, \
         { "relative_alt", NULL, MAVLINK_TYPE_INT32_T, 0, 36, offsetof(mavlink_camera_data_t, relative_alt) }, \
         { "hdg", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_camera_data_t, hdg) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_camera_data_t, fix_type) }, \
         { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_camera_data_t, satellites_visible) }, \
         } \
}


/**
 * @brief Pack a camera_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param time_week GPS week number
 * @param time_week_ms GPS time (milliseconds from start of GPS week)
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, int32_t time_week, uint32_t time_week_ms, float roll, float pitch, float yaw, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, uint16_t hdg, uint8_t fix_type, uint8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CAMERA_DATA_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, time_week);
	_mav_put_uint32_t(buf, 8, time_week_ms);
	_mav_put_float(buf, 12, roll);
	_mav_put_float(buf, 16, pitch);
	_mav_put_float(buf, 20, yaw);
	_mav_put_int32_t(buf, 24, lat);
	_mav_put_int32_t(buf, 28, lon);
	_mav_put_int32_t(buf, 32, alt);
	_mav_put_int32_t(buf, 36, relative_alt);
	_mav_put_uint16_t(buf, 40, hdg);
	_mav_put_uint8_t(buf, 42, fix_type);
	_mav_put_uint8_t(buf, 43, satellites_visible);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_DATA_LEN);
#else
	mavlink_camera_data_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.time_week = time_week;
	packet.time_week_ms = time_week_ms;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.hdg = hdg;
	packet.fix_type = fix_type;
	packet.satellites_visible = satellites_visible;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CAMERA_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAMERA_DATA_LEN, MAVLINK_MSG_ID_CAMERA_DATA_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CAMERA_DATA_LEN);
#endif
}

/**
 * @brief Pack a camera_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param time_week GPS week number
 * @param time_week_ms GPS time (milliseconds from start of GPS week)
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_camera_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,int32_t time_week,uint32_t time_week_ms,float roll,float pitch,float yaw,int32_t lat,int32_t lon,int32_t alt,int32_t relative_alt,uint16_t hdg,uint8_t fix_type,uint8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CAMERA_DATA_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, time_week);
	_mav_put_uint32_t(buf, 8, time_week_ms);
	_mav_put_float(buf, 12, roll);
	_mav_put_float(buf, 16, pitch);
	_mav_put_float(buf, 20, yaw);
	_mav_put_int32_t(buf, 24, lat);
	_mav_put_int32_t(buf, 28, lon);
	_mav_put_int32_t(buf, 32, alt);
	_mav_put_int32_t(buf, 36, relative_alt);
	_mav_put_uint16_t(buf, 40, hdg);
	_mav_put_uint8_t(buf, 42, fix_type);
	_mav_put_uint8_t(buf, 43, satellites_visible);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CAMERA_DATA_LEN);
#else
	mavlink_camera_data_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.time_week = time_week;
	packet.time_week_ms = time_week_ms;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.hdg = hdg;
	packet.fix_type = fix_type;
	packet.satellites_visible = satellites_visible;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CAMERA_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_CAMERA_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAMERA_DATA_LEN, MAVLINK_MSG_ID_CAMERA_DATA_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CAMERA_DATA_LEN);
#endif
}

/**
 * @brief Encode a camera_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param camera_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_camera_data_t* camera_data)
{
	return mavlink_msg_camera_data_pack(system_id, component_id, msg, camera_data->time_boot_ms, camera_data->time_week, camera_data->time_week_ms, camera_data->roll, camera_data->pitch, camera_data->yaw, camera_data->lat, camera_data->lon, camera_data->alt, camera_data->relative_alt, camera_data->hdg, camera_data->fix_type, camera_data->satellites_visible);
}

/**
 * @brief Encode a camera_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param camera_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_camera_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_camera_data_t* camera_data)
{
	return mavlink_msg_camera_data_pack_chan(system_id, component_id, chan, msg, camera_data->time_boot_ms, camera_data->time_week, camera_data->time_week_ms, camera_data->roll, camera_data->pitch, camera_data->yaw, camera_data->lat, camera_data->lon, camera_data->alt, camera_data->relative_alt, camera_data->hdg, camera_data->fix_type, camera_data->satellites_visible);
}

/**
 * @brief Send a camera_data message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param time_week GPS week number
 * @param time_week_ms GPS time (milliseconds from start of GPS week)
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), above MSL
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_camera_data_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t time_week, uint32_t time_week_ms, float roll, float pitch, float yaw, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, uint16_t hdg, uint8_t fix_type, uint8_t satellites_visible)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_CAMERA_DATA_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, time_week);
	_mav_put_uint32_t(buf, 8, time_week_ms);
	_mav_put_float(buf, 12, roll);
	_mav_put_float(buf, 16, pitch);
	_mav_put_float(buf, 20, yaw);
	_mav_put_int32_t(buf, 24, lat);
	_mav_put_int32_t(buf, 28, lon);
	_mav_put_int32_t(buf, 32, alt);
	_mav_put_int32_t(buf, 36, relative_alt);
	_mav_put_uint16_t(buf, 40, hdg);
	_mav_put_uint8_t(buf, 42, fix_type);
	_mav_put_uint8_t(buf, 43, satellites_visible);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_DATA, buf, MAVLINK_MSG_ID_CAMERA_DATA_LEN, MAVLINK_MSG_ID_CAMERA_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_DATA, buf, MAVLINK_MSG_ID_CAMERA_DATA_LEN);
#endif
#else
	mavlink_camera_data_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.time_week = time_week;
	packet.time_week_ms = time_week_ms;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;
	packet.relative_alt = relative_alt;
	packet.hdg = hdg;
	packet.fix_type = fix_type;
	packet.satellites_visible = satellites_visible;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_DATA, (const char *)&packet, MAVLINK_MSG_ID_CAMERA_DATA_LEN, MAVLINK_MSG_ID_CAMERA_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CAMERA_DATA, (const char *)&packet, MAVLINK_MSG_ID_CAMERA_DATA_LEN);
#endif
#endif
}

#endif

// MESSAGE CAMERA_DATA UNPACKING


/**
 * @brief Get field time_boot_ms from camera_data message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_camera_data_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field time_week from camera_data message
 *
 * @return GPS week number
 */
static inline int32_t mavlink_msg_camera_data_get_time_week(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field time_week_ms from camera_data message
 *
 * @return GPS time (milliseconds from start of GPS week)
 */
static inline uint32_t mavlink_msg_camera_data_get_time_week_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field roll from camera_data message
 *
 * @return Roll angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_camera_data_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pitch from camera_data message
 *
 * @return Pitch angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_camera_data_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field yaw from camera_data message
 *
 * @return Yaw angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_camera_data_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field lat from camera_data message
 *
 * @return Latitude, expressed as * 1E7
 */
static inline int32_t mavlink_msg_camera_data_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Get field lon from camera_data message
 *
 * @return Longitude, expressed as * 1E7
 */
static inline int32_t mavlink_msg_camera_data_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  28);
}

/**
 * @brief Get field alt from camera_data message
 *
 * @return Altitude in meters, expressed as * 1000 (millimeters), above MSL
 */
static inline int32_t mavlink_msg_camera_data_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  32);
}

/**
 * @brief Get field relative_alt from camera_data message
 *
 * @return Altitude above ground in meters, expressed as * 1000 (millimeters)
 */
static inline int32_t mavlink_msg_camera_data_get_relative_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  36);
}

/**
 * @brief Get field hdg from camera_data message
 *
 * @return Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 */
static inline uint16_t mavlink_msg_camera_data_get_hdg(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  40);
}

/**
 * @brief Get field fix_type from camera_data message
 *
 * @return 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 */
static inline uint8_t mavlink_msg_camera_data_get_fix_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field satellites_visible from camera_data message
 *
 * @return Number of satellites visible. If unknown, set to 255
 */
static inline uint8_t mavlink_msg_camera_data_get_satellites_visible(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  43);
}

/**
 * @brief Decode a camera_data message into a struct
 *
 * @param msg The message to decode
 * @param camera_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_camera_data_decode(const mavlink_message_t* msg, mavlink_camera_data_t* camera_data)
{
#if MAVLINK_NEED_BYTE_SWAP
	camera_data->time_boot_ms = mavlink_msg_camera_data_get_time_boot_ms(msg);
	camera_data->time_week = mavlink_msg_camera_data_get_time_week(msg);
	camera_data->time_week_ms = mavlink_msg_camera_data_get_time_week_ms(msg);
	camera_data->roll = mavlink_msg_camera_data_get_roll(msg);
	camera_data->pitch = mavlink_msg_camera_data_get_pitch(msg);
	camera_data->yaw = mavlink_msg_camera_data_get_yaw(msg);
	camera_data->lat = mavlink_msg_camera_data_get_lat(msg);
	camera_data->lon = mavlink_msg_camera_data_get_lon(msg);
	camera_data->alt = mavlink_msg_camera_data_get_alt(msg);
	camera_data->relative_alt = mavlink_msg_camera_data_get_relative_alt(msg);
	camera_data->hdg = mavlink_msg_camera_data_get_hdg(msg);
	camera_data->fix_type = mavlink_msg_camera_data_get_fix_type(msg);
	camera_data->satellites_visible = mavlink_msg_camera_data_get_satellites_visible(msg);
#else
	memcpy(camera_data, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_CAMERA_DATA_LEN);
#endif
}
