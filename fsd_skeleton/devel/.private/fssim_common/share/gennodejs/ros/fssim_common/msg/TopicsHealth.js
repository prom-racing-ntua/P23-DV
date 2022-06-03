// Auto-generated. Do not edit!

// (in-package fssim_common.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TopicState = require('./TopicState.js');

//-----------------------------------------------------------

class TopicsHealth {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.topics_check_passed = null;
      this.precision = null;
      this.topics_check = null;
    }
    else {
      if (initObj.hasOwnProperty('topics_check_passed')) {
        this.topics_check_passed = initObj.topics_check_passed
      }
      else {
        this.topics_check_passed = false;
      }
      if (initObj.hasOwnProperty('precision')) {
        this.precision = initObj.precision
      }
      else {
        this.precision = 0.0;
      }
      if (initObj.hasOwnProperty('topics_check')) {
        this.topics_check = initObj.topics_check
      }
      else {
        this.topics_check = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TopicsHealth
    // Serialize message field [topics_check_passed]
    bufferOffset = _serializer.bool(obj.topics_check_passed, buffer, bufferOffset);
    // Serialize message field [precision]
    bufferOffset = _serializer.float32(obj.precision, buffer, bufferOffset);
    // Serialize message field [topics_check]
    // Serialize the length for message field [topics_check]
    bufferOffset = _serializer.uint32(obj.topics_check.length, buffer, bufferOffset);
    obj.topics_check.forEach((val) => {
      bufferOffset = TopicState.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TopicsHealth
    let len;
    let data = new TopicsHealth(null);
    // Deserialize message field [topics_check_passed]
    data.topics_check_passed = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [precision]
    data.precision = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [topics_check]
    // Deserialize array length for message field [topics_check]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.topics_check = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.topics_check[i] = TopicState.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.topics_check.forEach((val) => {
      length += TopicState.getMessageSize(val);
    });
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fssim_common/TopicsHealth';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2e4a29cd88c13c0624f8c9a144bda96c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool topics_check_passed	# True is all topics passed check
    float32 precision			# How much we allow to deviate topics freq from expected
    TopicState[] topics_check  	# All topics health
    ================================================================================
    MSG: fssim_common/TopicState
    string topic_name
    float32 expected_frequency
    float32 measured_frequency
    bool passed
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TopicsHealth(null);
    if (msg.topics_check_passed !== undefined) {
      resolved.topics_check_passed = msg.topics_check_passed;
    }
    else {
      resolved.topics_check_passed = false
    }

    if (msg.precision !== undefined) {
      resolved.precision = msg.precision;
    }
    else {
      resolved.precision = 0.0
    }

    if (msg.topics_check !== undefined) {
      resolved.topics_check = new Array(msg.topics_check.length);
      for (let i = 0; i < resolved.topics_check.length; ++i) {
        resolved.topics_check[i] = TopicState.Resolve(msg.topics_check[i]);
      }
    }
    else {
      resolved.topics_check = []
    }

    return resolved;
    }
};

module.exports = TopicsHealth;
