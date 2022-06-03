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

class ResState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.emergency = null;
      this.on_off_switch = null;
      this.push_button = null;
      this.communication_interrupted = null;
    }
    else {
      if (initObj.hasOwnProperty('emergency')) {
        this.emergency = initObj.emergency
      }
      else {
        this.emergency = false;
      }
      if (initObj.hasOwnProperty('on_off_switch')) {
        this.on_off_switch = initObj.on_off_switch
      }
      else {
        this.on_off_switch = false;
      }
      if (initObj.hasOwnProperty('push_button')) {
        this.push_button = initObj.push_button
      }
      else {
        this.push_button = false;
      }
      if (initObj.hasOwnProperty('communication_interrupted')) {
        this.communication_interrupted = initObj.communication_interrupted
      }
      else {
        this.communication_interrupted = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ResState
    // Serialize message field [emergency]
    bufferOffset = _serializer.bool(obj.emergency, buffer, bufferOffset);
    // Serialize message field [on_off_switch]
    bufferOffset = _serializer.bool(obj.on_off_switch, buffer, bufferOffset);
    // Serialize message field [push_button]
    bufferOffset = _serializer.bool(obj.push_button, buffer, bufferOffset);
    // Serialize message field [communication_interrupted]
    bufferOffset = _serializer.bool(obj.communication_interrupted, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResState
    let len;
    let data = new ResState(null);
    // Deserialize message field [emergency]
    data.emergency = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [on_off_switch]
    data.on_off_switch = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [push_button]
    data.push_button = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [communication_interrupted]
    data.communication_interrupted = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fssim_common/ResState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2c68d942044efe0714c25879acd65327';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # State of the three buttons on the RES
    bool emergency 					# 0 = OK, 1 = Emergency
    bool on_off_switch 				# 0 = 0, 1 = 1 (Comments couldn't be more helpful :p )
    bool push_button 				# 1 = pressed
    # RES will trigger emergency 200ms after this becomes 1 (unless communication is recovered)
    bool communication_interrupted 	# 0 = OK, 1 = Interrupted
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ResState(null);
    if (msg.emergency !== undefined) {
      resolved.emergency = msg.emergency;
    }
    else {
      resolved.emergency = false
    }

    if (msg.on_off_switch !== undefined) {
      resolved.on_off_switch = msg.on_off_switch;
    }
    else {
      resolved.on_off_switch = false
    }

    if (msg.push_button !== undefined) {
      resolved.push_button = msg.push_button;
    }
    else {
      resolved.push_button = false
    }

    if (msg.communication_interrupted !== undefined) {
      resolved.communication_interrupted = msg.communication_interrupted;
    }
    else {
      resolved.communication_interrupted = false
    }

    return resolved;
    }
};

module.exports = ResState;
