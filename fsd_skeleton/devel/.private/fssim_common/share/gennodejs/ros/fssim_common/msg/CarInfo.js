// Auto-generated. Do not edit!

// (in-package fssim_common.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Vector3Ext = require('./Vector3Ext.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class CarInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.drag_force = null;
      this.delta = null;
      this.dc = null;
      this.front_left_steering_angle = null;
      this.front_right_steering_angle = null;
      this.delta_measured = null;
      this.vx = null;
      this.vy = null;
      this.r = null;
      this.torque_ok = null;
      this.alpha_f = null;
      this.alpha_f_l = null;
      this.alpha_f_r = null;
      this.alpha_r_l = null;
      this.alpha_r = null;
      this.alpha_r_r = null;
      this.Fy_f = null;
      this.Fy_f_l = null;
      this.Fy_f_r = null;
      this.Fy_r = null;
      this.Fy_r_l = null;
      this.Fy_r_r = null;
      this.Fx = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('drag_force')) {
        this.drag_force = initObj.drag_force
      }
      else {
        this.drag_force = new Vector3Ext();
      }
      if (initObj.hasOwnProperty('delta')) {
        this.delta = initObj.delta
      }
      else {
        this.delta = 0.0;
      }
      if (initObj.hasOwnProperty('dc')) {
        this.dc = initObj.dc
      }
      else {
        this.dc = 0.0;
      }
      if (initObj.hasOwnProperty('front_left_steering_angle')) {
        this.front_left_steering_angle = initObj.front_left_steering_angle
      }
      else {
        this.front_left_steering_angle = 0.0;
      }
      if (initObj.hasOwnProperty('front_right_steering_angle')) {
        this.front_right_steering_angle = initObj.front_right_steering_angle
      }
      else {
        this.front_right_steering_angle = 0.0;
      }
      if (initObj.hasOwnProperty('delta_measured')) {
        this.delta_measured = initObj.delta_measured
      }
      else {
        this.delta_measured = 0.0;
      }
      if (initObj.hasOwnProperty('vx')) {
        this.vx = initObj.vx
      }
      else {
        this.vx = 0.0;
      }
      if (initObj.hasOwnProperty('vy')) {
        this.vy = initObj.vy
      }
      else {
        this.vy = 0.0;
      }
      if (initObj.hasOwnProperty('r')) {
        this.r = initObj.r
      }
      else {
        this.r = 0.0;
      }
      if (initObj.hasOwnProperty('torque_ok')) {
        this.torque_ok = initObj.torque_ok
      }
      else {
        this.torque_ok = false;
      }
      if (initObj.hasOwnProperty('alpha_f')) {
        this.alpha_f = initObj.alpha_f
      }
      else {
        this.alpha_f = 0.0;
      }
      if (initObj.hasOwnProperty('alpha_f_l')) {
        this.alpha_f_l = initObj.alpha_f_l
      }
      else {
        this.alpha_f_l = 0.0;
      }
      if (initObj.hasOwnProperty('alpha_f_r')) {
        this.alpha_f_r = initObj.alpha_f_r
      }
      else {
        this.alpha_f_r = 0.0;
      }
      if (initObj.hasOwnProperty('alpha_r_l')) {
        this.alpha_r_l = initObj.alpha_r_l
      }
      else {
        this.alpha_r_l = 0.0;
      }
      if (initObj.hasOwnProperty('alpha_r')) {
        this.alpha_r = initObj.alpha_r
      }
      else {
        this.alpha_r = 0.0;
      }
      if (initObj.hasOwnProperty('alpha_r_r')) {
        this.alpha_r_r = initObj.alpha_r_r
      }
      else {
        this.alpha_r_r = 0.0;
      }
      if (initObj.hasOwnProperty('Fy_f')) {
        this.Fy_f = initObj.Fy_f
      }
      else {
        this.Fy_f = 0.0;
      }
      if (initObj.hasOwnProperty('Fy_f_l')) {
        this.Fy_f_l = initObj.Fy_f_l
      }
      else {
        this.Fy_f_l = 0.0;
      }
      if (initObj.hasOwnProperty('Fy_f_r')) {
        this.Fy_f_r = initObj.Fy_f_r
      }
      else {
        this.Fy_f_r = 0.0;
      }
      if (initObj.hasOwnProperty('Fy_r')) {
        this.Fy_r = initObj.Fy_r
      }
      else {
        this.Fy_r = 0.0;
      }
      if (initObj.hasOwnProperty('Fy_r_l')) {
        this.Fy_r_l = initObj.Fy_r_l
      }
      else {
        this.Fy_r_l = 0.0;
      }
      if (initObj.hasOwnProperty('Fy_r_r')) {
        this.Fy_r_r = initObj.Fy_r_r
      }
      else {
        this.Fy_r_r = 0.0;
      }
      if (initObj.hasOwnProperty('Fx')) {
        this.Fx = initObj.Fx
      }
      else {
        this.Fx = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CarInfo
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [drag_force]
    bufferOffset = Vector3Ext.serialize(obj.drag_force, buffer, bufferOffset);
    // Serialize message field [delta]
    bufferOffset = _serializer.float64(obj.delta, buffer, bufferOffset);
    // Serialize message field [dc]
    bufferOffset = _serializer.float64(obj.dc, buffer, bufferOffset);
    // Serialize message field [front_left_steering_angle]
    bufferOffset = _serializer.float64(obj.front_left_steering_angle, buffer, bufferOffset);
    // Serialize message field [front_right_steering_angle]
    bufferOffset = _serializer.float64(obj.front_right_steering_angle, buffer, bufferOffset);
    // Serialize message field [delta_measured]
    bufferOffset = _serializer.float64(obj.delta_measured, buffer, bufferOffset);
    // Serialize message field [vx]
    bufferOffset = _serializer.float64(obj.vx, buffer, bufferOffset);
    // Serialize message field [vy]
    bufferOffset = _serializer.float64(obj.vy, buffer, bufferOffset);
    // Serialize message field [r]
    bufferOffset = _serializer.float64(obj.r, buffer, bufferOffset);
    // Serialize message field [torque_ok]
    bufferOffset = _serializer.bool(obj.torque_ok, buffer, bufferOffset);
    // Serialize message field [alpha_f]
    bufferOffset = _serializer.float64(obj.alpha_f, buffer, bufferOffset);
    // Serialize message field [alpha_f_l]
    bufferOffset = _serializer.float64(obj.alpha_f_l, buffer, bufferOffset);
    // Serialize message field [alpha_f_r]
    bufferOffset = _serializer.float64(obj.alpha_f_r, buffer, bufferOffset);
    // Serialize message field [alpha_r_l]
    bufferOffset = _serializer.float64(obj.alpha_r_l, buffer, bufferOffset);
    // Serialize message field [alpha_r]
    bufferOffset = _serializer.float64(obj.alpha_r, buffer, bufferOffset);
    // Serialize message field [alpha_r_r]
    bufferOffset = _serializer.float64(obj.alpha_r_r, buffer, bufferOffset);
    // Serialize message field [Fy_f]
    bufferOffset = _serializer.float64(obj.Fy_f, buffer, bufferOffset);
    // Serialize message field [Fy_f_l]
    bufferOffset = _serializer.float64(obj.Fy_f_l, buffer, bufferOffset);
    // Serialize message field [Fy_f_r]
    bufferOffset = _serializer.float64(obj.Fy_f_r, buffer, bufferOffset);
    // Serialize message field [Fy_r]
    bufferOffset = _serializer.float64(obj.Fy_r, buffer, bufferOffset);
    // Serialize message field [Fy_r_l]
    bufferOffset = _serializer.float64(obj.Fy_r_l, buffer, bufferOffset);
    // Serialize message field [Fy_r_r]
    bufferOffset = _serializer.float64(obj.Fy_r_r, buffer, bufferOffset);
    // Serialize message field [Fx]
    bufferOffset = _serializer.float64(obj.Fx, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CarInfo
    let len;
    let data = new CarInfo(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [drag_force]
    data.drag_force = Vector3Ext.deserialize(buffer, bufferOffset);
    // Deserialize message field [delta]
    data.delta = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [dc]
    data.dc = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [front_left_steering_angle]
    data.front_left_steering_angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [front_right_steering_angle]
    data.front_right_steering_angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [delta_measured]
    data.delta_measured = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vx]
    data.vx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vy]
    data.vy = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [r]
    data.r = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [torque_ok]
    data.torque_ok = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [alpha_f]
    data.alpha_f = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [alpha_f_l]
    data.alpha_f_l = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [alpha_f_r]
    data.alpha_f_r = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [alpha_r_l]
    data.alpha_r_l = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [alpha_r]
    data.alpha_r = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [alpha_r_r]
    data.alpha_r_r = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Fy_f]
    data.Fy_f = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Fy_f_l]
    data.Fy_f_l = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Fy_f_r]
    data.Fy_f_r = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Fy_r]
    data.Fy_r = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Fy_r_l]
    data.Fy_r_l = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Fy_r_r]
    data.Fy_r_r = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Fx]
    data.Fx = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 201;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fssim_common/CarInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd1d7b96c5e9f10a89a35df8f3f330a2c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    Vector3Ext drag_force
    
    float64 delta
    float64 dc
    
    float64 front_left_steering_angle
    float64 front_right_steering_angle
    float64 delta_measured
    
    float64 vx
    float64 vy
    float64 r
    
    bool torque_ok 
    
    float64 alpha_f
    float64 alpha_f_l
    float64 alpha_f_r
    float64 alpha_r_l
    float64 alpha_r
    float64 alpha_r_r
    
    float64 Fy_f
    float64 Fy_f_l
    float64 Fy_f_r
    float64 Fy_r
    float64 Fy_r_l
    float64 Fy_r_r
    
    float64 Fx
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
    MSG: fssim_common/Vector3Ext
    geometry_msgs/Vector3 vec
    float64 mag
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
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
    const resolved = new CarInfo(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.drag_force !== undefined) {
      resolved.drag_force = Vector3Ext.Resolve(msg.drag_force)
    }
    else {
      resolved.drag_force = new Vector3Ext()
    }

    if (msg.delta !== undefined) {
      resolved.delta = msg.delta;
    }
    else {
      resolved.delta = 0.0
    }

    if (msg.dc !== undefined) {
      resolved.dc = msg.dc;
    }
    else {
      resolved.dc = 0.0
    }

    if (msg.front_left_steering_angle !== undefined) {
      resolved.front_left_steering_angle = msg.front_left_steering_angle;
    }
    else {
      resolved.front_left_steering_angle = 0.0
    }

    if (msg.front_right_steering_angle !== undefined) {
      resolved.front_right_steering_angle = msg.front_right_steering_angle;
    }
    else {
      resolved.front_right_steering_angle = 0.0
    }

    if (msg.delta_measured !== undefined) {
      resolved.delta_measured = msg.delta_measured;
    }
    else {
      resolved.delta_measured = 0.0
    }

    if (msg.vx !== undefined) {
      resolved.vx = msg.vx;
    }
    else {
      resolved.vx = 0.0
    }

    if (msg.vy !== undefined) {
      resolved.vy = msg.vy;
    }
    else {
      resolved.vy = 0.0
    }

    if (msg.r !== undefined) {
      resolved.r = msg.r;
    }
    else {
      resolved.r = 0.0
    }

    if (msg.torque_ok !== undefined) {
      resolved.torque_ok = msg.torque_ok;
    }
    else {
      resolved.torque_ok = false
    }

    if (msg.alpha_f !== undefined) {
      resolved.alpha_f = msg.alpha_f;
    }
    else {
      resolved.alpha_f = 0.0
    }

    if (msg.alpha_f_l !== undefined) {
      resolved.alpha_f_l = msg.alpha_f_l;
    }
    else {
      resolved.alpha_f_l = 0.0
    }

    if (msg.alpha_f_r !== undefined) {
      resolved.alpha_f_r = msg.alpha_f_r;
    }
    else {
      resolved.alpha_f_r = 0.0
    }

    if (msg.alpha_r_l !== undefined) {
      resolved.alpha_r_l = msg.alpha_r_l;
    }
    else {
      resolved.alpha_r_l = 0.0
    }

    if (msg.alpha_r !== undefined) {
      resolved.alpha_r = msg.alpha_r;
    }
    else {
      resolved.alpha_r = 0.0
    }

    if (msg.alpha_r_r !== undefined) {
      resolved.alpha_r_r = msg.alpha_r_r;
    }
    else {
      resolved.alpha_r_r = 0.0
    }

    if (msg.Fy_f !== undefined) {
      resolved.Fy_f = msg.Fy_f;
    }
    else {
      resolved.Fy_f = 0.0
    }

    if (msg.Fy_f_l !== undefined) {
      resolved.Fy_f_l = msg.Fy_f_l;
    }
    else {
      resolved.Fy_f_l = 0.0
    }

    if (msg.Fy_f_r !== undefined) {
      resolved.Fy_f_r = msg.Fy_f_r;
    }
    else {
      resolved.Fy_f_r = 0.0
    }

    if (msg.Fy_r !== undefined) {
      resolved.Fy_r = msg.Fy_r;
    }
    else {
      resolved.Fy_r = 0.0
    }

    if (msg.Fy_r_l !== undefined) {
      resolved.Fy_r_l = msg.Fy_r_l;
    }
    else {
      resolved.Fy_r_l = 0.0
    }

    if (msg.Fy_r_r !== undefined) {
      resolved.Fy_r_r = msg.Fy_r_r;
    }
    else {
      resolved.Fy_r_r = 0.0
    }

    if (msg.Fx !== undefined) {
      resolved.Fx = msg.Fx;
    }
    else {
      resolved.Fx = 0.0
    }

    return resolved;
    }
};

module.exports = CarInfo;
