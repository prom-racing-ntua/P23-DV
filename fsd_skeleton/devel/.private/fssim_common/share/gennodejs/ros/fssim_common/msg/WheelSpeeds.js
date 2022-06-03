// Auto-generated. Do not edit!

// (in-package fssim_common.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class WheelSpeeds {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.version = null;
      this.rpm_front_left = null;
      this.rpm_front_right = null;
      this.rpm_rear_left = null;
      this.rpm_rear_right = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('version')) {
        this.version = initObj.version
      }
      else {
        this.version = 0;
      }
      if (initObj.hasOwnProperty('rpm_front_left')) {
        this.rpm_front_left = initObj.rpm_front_left
      }
      else {
        this.rpm_front_left = 0;
      }
      if (initObj.hasOwnProperty('rpm_front_right')) {
        this.rpm_front_right = initObj.rpm_front_right
      }
      else {
        this.rpm_front_right = 0;
      }
      if (initObj.hasOwnProperty('rpm_rear_left')) {
        this.rpm_rear_left = initObj.rpm_rear_left
      }
      else {
        this.rpm_rear_left = 0;
      }
      if (initObj.hasOwnProperty('rpm_rear_right')) {
        this.rpm_rear_right = initObj.rpm_rear_right
      }
      else {
        this.rpm_rear_right = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WheelSpeeds
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [version]
    bufferOffset = _serializer.uint8(obj.version, buffer, bufferOffset);
    // Serialize message field [rpm_front_left]
    bufferOffset = _serializer.int16(obj.rpm_front_left, buffer, bufferOffset);
    // Serialize message field [rpm_front_right]
    bufferOffset = _serializer.int16(obj.rpm_front_right, buffer, bufferOffset);
    // Serialize message field [rpm_rear_left]
    bufferOffset = _serializer.int16(obj.rpm_rear_left, buffer, bufferOffset);
    // Serialize message field [rpm_rear_right]
    bufferOffset = _serializer.int16(obj.rpm_rear_right, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WheelSpeeds
    let len;
    let data = new WheelSpeeds(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [version]
    data.version = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [rpm_front_left]
    data.rpm_front_left = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [rpm_front_right]
    data.rpm_front_right = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [rpm_rear_left]
    data.rpm_rear_left = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [rpm_rear_right]
    data.rpm_rear_right = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fssim_common/WheelSpeeds';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'de6e76c895b1095f899172fc46f64a60';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Header
    std_msgs/Header header
    uint8 version
    
    int16 rpm_front_left
    int16 rpm_front_right
    int16 rpm_rear_left
    int16 rpm_rear_right
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WheelSpeeds(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.version !== undefined) {
      resolved.version = msg.version;
    }
    else {
      resolved.version = 0
    }

    if (msg.rpm_front_left !== undefined) {
      resolved.rpm_front_left = msg.rpm_front_left;
    }
    else {
      resolved.rpm_front_left = 0
    }

    if (msg.rpm_front_right !== undefined) {
      resolved.rpm_front_right = msg.rpm_front_right;
    }
    else {
      resolved.rpm_front_right = 0
    }

    if (msg.rpm_rear_left !== undefined) {
      resolved.rpm_rear_left = msg.rpm_rear_left;
    }
    else {
      resolved.rpm_rear_left = 0
    }

    if (msg.rpm_rear_right !== undefined) {
      resolved.rpm_rear_right = msg.rpm_rear_right;
    }
    else {
      resolved.rpm_rear_right = 0
    }

    return resolved;
    }
};

module.exports = WheelSpeeds;
