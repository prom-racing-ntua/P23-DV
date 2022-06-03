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

class TopicState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.topic_name = null;
      this.expected_frequency = null;
      this.measured_frequency = null;
      this.passed = null;
    }
    else {
      if (initObj.hasOwnProperty('topic_name')) {
        this.topic_name = initObj.topic_name
      }
      else {
        this.topic_name = '';
      }
      if (initObj.hasOwnProperty('expected_frequency')) {
        this.expected_frequency = initObj.expected_frequency
      }
      else {
        this.expected_frequency = 0.0;
      }
      if (initObj.hasOwnProperty('measured_frequency')) {
        this.measured_frequency = initObj.measured_frequency
      }
      else {
        this.measured_frequency = 0.0;
      }
      if (initObj.hasOwnProperty('passed')) {
        this.passed = initObj.passed
      }
      else {
        this.passed = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TopicState
    // Serialize message field [topic_name]
    bufferOffset = _serializer.string(obj.topic_name, buffer, bufferOffset);
    // Serialize message field [expected_frequency]
    bufferOffset = _serializer.float32(obj.expected_frequency, buffer, bufferOffset);
    // Serialize message field [measured_frequency]
    bufferOffset = _serializer.float32(obj.measured_frequency, buffer, bufferOffset);
    // Serialize message field [passed]
    bufferOffset = _serializer.bool(obj.passed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TopicState
    let len;
    let data = new TopicState(null);
    // Deserialize message field [topic_name]
    data.topic_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [expected_frequency]
    data.expected_frequency = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [measured_frequency]
    data.measured_frequency = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [passed]
    data.passed = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.topic_name);
    return length + 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fssim_common/TopicState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5557167df4d3920fba79516729b9f245';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new TopicState(null);
    if (msg.topic_name !== undefined) {
      resolved.topic_name = msg.topic_name;
    }
    else {
      resolved.topic_name = ''
    }

    if (msg.expected_frequency !== undefined) {
      resolved.expected_frequency = msg.expected_frequency;
    }
    else {
      resolved.expected_frequency = 0.0
    }

    if (msg.measured_frequency !== undefined) {
      resolved.measured_frequency = msg.measured_frequency;
    }
    else {
      resolved.measured_frequency = 0.0
    }

    if (msg.passed !== undefined) {
      resolved.passed = msg.passed;
    }
    else {
      resolved.passed = false
    }

    return resolved;
    }
};

module.exports = TopicState;
