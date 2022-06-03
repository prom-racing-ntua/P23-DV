// Auto-generated. Do not edit!

// (in-package fssim_common.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Cmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dc = null;
      this.delta = null;
    }
    else {
      if (initObj.hasOwnProperty('dc')) {
        this.dc = initObj.dc
      }
      else {
        this.dc = 0.0;
      }
      if (initObj.hasOwnProperty('delta')) {
        this.delta = initObj.delta
      }
      else {
        this.delta = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Cmd
    // Serialize message field [dc]
    bufferOffset = _serializer.float64(obj.dc, buffer, bufferOffset);
    // Serialize message field [delta]
    bufferOffset = _serializer.float64(obj.delta, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Cmd
    let len;
    let data = new Cmd(null);
    // Deserialize message field [dc]
    data.dc = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [delta]
    data.delta = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fssim_common/Cmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7c30c8d10cd397c67459b00d587e06f1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 dc
    float64 delta
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Cmd(null);
    if (msg.dc !== undefined) {
      resolved.dc = msg.dc;
    }
    else {
      resolved.dc = 0.0
    }

    if (msg.delta !== undefined) {
      resolved.delta = msg.delta;
    }
    else {
      resolved.delta = 0.0
    }

    return resolved;
    }
};

module.exports = Cmd;
