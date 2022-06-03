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

//-----------------------------------------------------------

class Vector3Ext {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vec = null;
      this.mag = null;
    }
    else {
      if (initObj.hasOwnProperty('vec')) {
        this.vec = initObj.vec
      }
      else {
        this.vec = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('mag')) {
        this.mag = initObj.mag
      }
      else {
        this.mag = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Vector3Ext
    // Serialize message field [vec]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.vec, buffer, bufferOffset);
    // Serialize message field [mag]
    bufferOffset = _serializer.float64(obj.mag, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Vector3Ext
    let len;
    let data = new Vector3Ext(null);
    // Deserialize message field [vec]
    data.vec = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [mag]
    data.mag = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fssim_common/Vector3Ext';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9ce7c2ac065b8145af6ba1b53af0b0bd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Vector3Ext(null);
    if (msg.vec !== undefined) {
      resolved.vec = geometry_msgs.msg.Vector3.Resolve(msg.vec)
    }
    else {
      resolved.vec = new geometry_msgs.msg.Vector3()
    }

    if (msg.mag !== undefined) {
      resolved.mag = msg.mag;
    }
    else {
      resolved.mag = 0.0
    }

    return resolved;
    }
};

module.exports = Vector3Ext;
