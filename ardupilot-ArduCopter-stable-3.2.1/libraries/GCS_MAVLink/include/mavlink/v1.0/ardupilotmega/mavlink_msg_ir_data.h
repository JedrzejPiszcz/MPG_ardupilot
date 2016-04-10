// MESSAGE IR_DATA PACKING

#define MAVLINK_MSG_ID_IR_DATA 227

typedef struct __mavlink_ir_data_t
{
 float x_position; /*< pozycja x*/
 float y_position; /*< pozycja y*/
} mavlink_ir_data_t;

#define MAVLINK_MSG_ID_IR_DATA_LEN 8
#define MAVLINK_MSG_ID_227_LEN 8

#define MAVLINK_MSG_ID_IR_DATA_CRC 52
#define MAVLINK_MSG_ID_227_CRC 52



#define MAVLINK_MESSAGE_INFO_IR_DATA { \
	"IR_DATA", \
	2, \
	{  { "x_position", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_ir_data_t, x_position) }, \
         { "y_position", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_ir_data_t, y_position) }, \
         } \
}


/**
 * @brief Pack a ir_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x_position pozycja x
 * @param y_position pozycja y
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ir_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float x_position, float y_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_IR_DATA_LEN];
	_mav_put_float(buf, 0, x_position);
	_mav_put_float(buf, 4, y_position);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IR_DATA_LEN);
#else
	mavlink_ir_data_t packet;
	packet.x_position = x_position;
	packet.y_position = y_position;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IR_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_IR_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IR_DATA_LEN, MAVLINK_MSG_ID_IR_DATA_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IR_DATA_LEN);
#endif
}

/**
 * @brief Pack a ir_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param x_position pozycja x
 * @param y_position pozycja y
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ir_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float x_position,float y_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_IR_DATA_LEN];
	_mav_put_float(buf, 0, x_position);
	_mav_put_float(buf, 4, y_position);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IR_DATA_LEN);
#else
	mavlink_ir_data_t packet;
	packet.x_position = x_position;
	packet.y_position = y_position;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IR_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_IR_DATA;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IR_DATA_LEN, MAVLINK_MSG_ID_IR_DATA_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IR_DATA_LEN);
#endif
}

/**
 * @brief Encode a ir_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ir_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ir_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ir_data_t* ir_data)
{
	return mavlink_msg_ir_data_pack(system_id, component_id, msg, ir_data->x_position, ir_data->y_position);
}

/**
 * @brief Encode a ir_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ir_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ir_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ir_data_t* ir_data)
{
	return mavlink_msg_ir_data_pack_chan(system_id, component_id, chan, msg, ir_data->x_position, ir_data->y_position);
}

/**
 * @brief Send a ir_data message
 * @param chan MAVLink channel to send the message
 *
 * @param x_position pozycja x
 * @param y_position pozycja y
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ir_data_send(mavlink_channel_t chan, float x_position, float y_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_IR_DATA_LEN];
	_mav_put_float(buf, 0, x_position);
	_mav_put_float(buf, 4, y_position);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IR_DATA, buf, MAVLINK_MSG_ID_IR_DATA_LEN, MAVLINK_MSG_ID_IR_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IR_DATA, buf, MAVLINK_MSG_ID_IR_DATA_LEN);
#endif
#else
	mavlink_ir_data_t packet;
	packet.x_position = x_position;
	packet.y_position = y_position;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IR_DATA, (const char *)&packet, MAVLINK_MSG_ID_IR_DATA_LEN, MAVLINK_MSG_ID_IR_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IR_DATA, (const char *)&packet, MAVLINK_MSG_ID_IR_DATA_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_IR_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ir_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x_position, float y_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, x_position);
	_mav_put_float(buf, 4, y_position);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IR_DATA, buf, MAVLINK_MSG_ID_IR_DATA_LEN, MAVLINK_MSG_ID_IR_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IR_DATA, buf, MAVLINK_MSG_ID_IR_DATA_LEN);
#endif
#else
	mavlink_ir_data_t *packet = (mavlink_ir_data_t *)msgbuf;
	packet->x_position = x_position;
	packet->y_position = y_position;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IR_DATA, (const char *)packet, MAVLINK_MSG_ID_IR_DATA_LEN, MAVLINK_MSG_ID_IR_DATA_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IR_DATA, (const char *)packet, MAVLINK_MSG_ID_IR_DATA_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE IR_DATA UNPACKING


/**
 * @brief Get field x_position from ir_data message
 *
 * @return pozycja x
 */
static inline float mavlink_msg_ir_data_get_x_position(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y_position from ir_data message
 *
 * @return pozycja y
 */
static inline float mavlink_msg_ir_data_get_y_position(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a ir_data message into a struct
 *
 * @param msg The message to decode
 * @param ir_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_ir_data_decode(const mavlink_message_t* msg, mavlink_ir_data_t* ir_data)
{
#if MAVLINK_NEED_BYTE_SWAP
	ir_data->x_position = mavlink_msg_ir_data_get_x_position(msg);
	ir_data->y_position = mavlink_msg_ir_data_get_y_position(msg);
#else
	memcpy(ir_data, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_IR_DATA_LEN);
#endif
}
