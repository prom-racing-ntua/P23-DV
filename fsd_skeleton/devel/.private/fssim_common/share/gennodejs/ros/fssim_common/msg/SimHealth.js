// Auto-generated. Do not edit!

// (in-package fssim_common.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TopicsHealth = require('./TopicsHealth.js');

//-----------------------------------------------------------

class SimHealth {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.request_shutdown = null;
      this.vehicle_started = null;
      this.topics_health = null;
    }
    else {
      if (initObj.hasOwnProperty('request_shutdown')) {
        this.request_shutdown = initObj.request_shutdown
      }
      else {
        this.request_shutdown = false;
      }
      if (initObj.hasOwnProperty('vehicle_started')) {
        this.vehicle_started = initObj.vehicle_started
      }
      else {
        this.vehicle_started = false;
      }
      if (initObj.hasOwnProperty('topics_health')) {
        this.topics_health = initObj.topics_health
      }
      else {
        this.topics_health = new TopicsHealth();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SimHealth
    // Serialize message field [request_shutdown]
    bufferOffset = _serializer.bool(obj.request_shutdown, buffer, bufferOffset);
    // Serialize message field [vehicle_started]
    bufferOffset = _serializer.bool(obj.vehicle_started, buffer, bufferOffset);
    // Serialize message field [topics_health]
    bufferOffset = TopicsHealth.serialize(obj.topics_health, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SimHealth
    let len;
    let data = new SimHealth(null);
    // Deserialize message field [request_shutdown]
    data.request_shutdown = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [vehicle_started]
    data.vehicle_started = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [topics_health]
    data.topics_health = TopicsHealth.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += TopicsHealth.getMessageSize(object.topics_health);
    return length + 2;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fssim_common/SimHealth';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2d6de42391271dba371094f7524b84b6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool request_shutdown 	# If TRUE we request immidiate shutdown
    bool vehicle_started	# We send res message to start the vehicle	
    
    TopicsHealth topics_health # List of topics and their health
    ================================================================================
    MSG: fssim_common/TopicsHealth
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
    const resolved = new SimHealth(null);
    if (msg.request_shutdown !== undefined) {
      resolved.request_shutdown = msg.request_shutdown;
    }
    else {
      resolved.request_shutdown = false
    }

    if (msg.vehicle_started !== undefined) {
      resolved.vehicle_started = msg.vehicle_started;
    }
    else {
      resolved.vehicle_started = false
    }

    if (msg.topics_health !== undefined) {
      resolved.topics_health = TopicsHealth.Resolve(msg.topics_health)
    }
    else {
      resolved.topics_health = new TopicsHealth()
    }

    return resolved;
    }
};

module.exports = SimHealth;
