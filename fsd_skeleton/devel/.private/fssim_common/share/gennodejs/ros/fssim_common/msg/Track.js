// Auto-generated. Do not edit!

// (in-package fssim_common.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Track {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.cones_left = null;
      this.cones_right = null;
      this.cones_orange = null;
      this.cones_orange_big = null;
      this.tk_device_start = null;
      this.tk_device_end = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('cones_left')) {
        this.cones_left = initObj.cones_left
      }
      else {
        this.cones_left = [];
      }
      if (initObj.hasOwnProperty('cones_right')) {
        this.cones_right = initObj.cones_right
      }
      else {
        this.cones_right = [];
      }
      if (initObj.hasOwnProperty('cones_orange')) {
        this.cones_orange = initObj.cones_orange
      }
      else {
        this.cones_orange = [];
      }
      if (initObj.hasOwnProperty('cones_orange_big')) {
        this.cones_orange_big = initObj.cones_orange_big
      }
      else {
        this.cones_orange_big = [];
      }
      if (initObj.hasOwnProperty('tk_device_start')) {
        this.tk_device_start = initObj.tk_device_start
      }
      else {
        this.tk_device_start = [];
      }
      if (initObj.hasOwnProperty('tk_device_end')) {
        this.tk_device_end = initObj.tk_device_end
      }
      else {
        this.tk_device_end = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Track
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [cones_left]
    // Serialize the length for message field [cones_left]
    bufferOffset = _serializer.uint32(obj.cones_left.length, buffer, bufferOffset);
    obj.cones_left.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [cones_right]
    // Serialize the length for message field [cones_right]
    bufferOffset = _serializer.uint32(obj.cones_right.length, buffer, bufferOffset);
    obj.cones_right.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [cones_orange]
    // Serialize the length for message field [cones_orange]
    bufferOffset = _serializer.uint32(obj.cones_orange.length, buffer, bufferOffset);
    obj.cones_orange.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [cones_orange_big]
    // Serialize the length for message field [cones_orange_big]
    bufferOffset = _serializer.uint32(obj.cones_orange_big.length, buffer, bufferOffset);
    obj.cones_orange_big.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [tk_device_start]
    // Serialize the length for message field [tk_device_start]
    bufferOffset = _serializer.uint32(obj.tk_device_start.length, buffer, bufferOffset);
    obj.tk_device_start.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [tk_device_end]
    // Serialize the length for message field [tk_device_end]
    bufferOffset = _serializer.uint32(obj.tk_device_end.length, buffer, bufferOffset);
    obj.tk_device_end.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Track
    let len;
    let data = new Track(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [cones_left]
    // Deserialize array length for message field [cones_left]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.cones_left = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.cones_left[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [cones_right]
    // Deserialize array length for message field [cones_right]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.cones_right = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.cones_right[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [cones_orange]
    // Deserialize array length for message field [cones_orange]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.cones_orange = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.cones_orange[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [cones_orange_big]
    // Deserialize array length for message field [cones_orange_big]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.cones_orange_big = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.cones_orange_big[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [tk_device_start]
    // Deserialize array length for message field [tk_device_start]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.tk_device_start = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.tk_device_start[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [tk_device_end]
    // Deserialize array length for message field [tk_device_end]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.tk_device_end = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.tk_device_end[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 24 * object.cones_left.length;
    length += 24 * object.cones_right.length;
    length += 24 * object.cones_orange.length;
    length += 24 * object.cones_orange_big.length;
    length += 24 * object.tk_device_start.length;
    length += 24 * object.tk_device_end.length;
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fssim_common/Track';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b3e47dce05b0e0c04dc61079e1408cdf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    geometry_msgs/Point[] cones_left
    geometry_msgs/Point[] cones_right
    geometry_msgs/Point[] cones_orange
    geometry_msgs/Point[] cones_orange_big
    geometry_msgs/Point[] tk_device_start
    geometry_msgs/Point[] tk_device_end
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
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Track(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.cones_left !== undefined) {
      resolved.cones_left = new Array(msg.cones_left.length);
      for (let i = 0; i < resolved.cones_left.length; ++i) {
        resolved.cones_left[i] = geometry_msgs.msg.Point.Resolve(msg.cones_left[i]);
      }
    }
    else {
      resolved.cones_left = []
    }

    if (msg.cones_right !== undefined) {
      resolved.cones_right = new Array(msg.cones_right.length);
      for (let i = 0; i < resolved.cones_right.length; ++i) {
        resolved.cones_right[i] = geometry_msgs.msg.Point.Resolve(msg.cones_right[i]);
      }
    }
    else {
      resolved.cones_right = []
    }

    if (msg.cones_orange !== undefined) {
      resolved.cones_orange = new Array(msg.cones_orange.length);
      for (let i = 0; i < resolved.cones_orange.length; ++i) {
        resolved.cones_orange[i] = geometry_msgs.msg.Point.Resolve(msg.cones_orange[i]);
      }
    }
    else {
      resolved.cones_orange = []
    }

    if (msg.cones_orange_big !== undefined) {
      resolved.cones_orange_big = new Array(msg.cones_orange_big.length);
      for (let i = 0; i < resolved.cones_orange_big.length; ++i) {
        resolved.cones_orange_big[i] = geometry_msgs.msg.Point.Resolve(msg.cones_orange_big[i]);
      }
    }
    else {
      resolved.cones_orange_big = []
    }

    if (msg.tk_device_start !== undefined) {
      resolved.tk_device_start = new Array(msg.tk_device_start.length);
      for (let i = 0; i < resolved.tk_device_start.length; ++i) {
        resolved.tk_device_start[i] = geometry_msgs.msg.Point.Resolve(msg.tk_device_start[i]);
      }
    }
    else {
      resolved.tk_device_start = []
    }

    if (msg.tk_device_end !== undefined) {
      resolved.tk_device_end = new Array(msg.tk_device_end.length);
      for (let i = 0; i < resolved.tk_device_end.length; ++i) {
        resolved.tk_device_end[i] = geometry_msgs.msg.Point.Resolve(msg.tk_device_end[i]);
      }
    }
    else {
      resolved.tk_device_end = []
    }

    return resolved;
    }
};

module.exports = Track;
