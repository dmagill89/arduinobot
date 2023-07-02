// Auto-generated. Do not edit!

// (in-package arduinobot_controller.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class AnglesConverterRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.base = null;
      this.shoulder = null;
      this.elbow = null;
      this.gripper = null;
    }
    else {
      if (initObj.hasOwnProperty('base')) {
        this.base = initObj.base
      }
      else {
        this.base = 0.0;
      }
      if (initObj.hasOwnProperty('shoulder')) {
        this.shoulder = initObj.shoulder
      }
      else {
        this.shoulder = 0.0;
      }
      if (initObj.hasOwnProperty('elbow')) {
        this.elbow = initObj.elbow
      }
      else {
        this.elbow = 0.0;
      }
      if (initObj.hasOwnProperty('gripper')) {
        this.gripper = initObj.gripper
      }
      else {
        this.gripper = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AnglesConverterRequest
    // Serialize message field [base]
    bufferOffset = _serializer.float64(obj.base, buffer, bufferOffset);
    // Serialize message field [shoulder]
    bufferOffset = _serializer.float64(obj.shoulder, buffer, bufferOffset);
    // Serialize message field [elbow]
    bufferOffset = _serializer.float64(obj.elbow, buffer, bufferOffset);
    // Serialize message field [gripper]
    bufferOffset = _serializer.float64(obj.gripper, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AnglesConverterRequest
    let len;
    let data = new AnglesConverterRequest(null);
    // Deserialize message field [base]
    data.base = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [shoulder]
    data.shoulder = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [elbow]
    data.elbow = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gripper]
    data.gripper = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a service object
    return 'arduinobot_controller/AnglesConverterRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '19b04e0ebe735f18fbbd5aea9d85b833';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Request 
    float64 base
    float64 shoulder
    float64 elbow
    float64 gripper
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AnglesConverterRequest(null);
    if (msg.base !== undefined) {
      resolved.base = msg.base;
    }
    else {
      resolved.base = 0.0
    }

    if (msg.shoulder !== undefined) {
      resolved.shoulder = msg.shoulder;
    }
    else {
      resolved.shoulder = 0.0
    }

    if (msg.elbow !== undefined) {
      resolved.elbow = msg.elbow;
    }
    else {
      resolved.elbow = 0.0
    }

    if (msg.gripper !== undefined) {
      resolved.gripper = msg.gripper;
    }
    else {
      resolved.gripper = 0.0
    }

    return resolved;
    }
};

class AnglesConverterResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.base = null;
      this.shoulder = null;
      this.elbow = null;
      this.gripper = null;
    }
    else {
      if (initObj.hasOwnProperty('base')) {
        this.base = initObj.base
      }
      else {
        this.base = 0.0;
      }
      if (initObj.hasOwnProperty('shoulder')) {
        this.shoulder = initObj.shoulder
      }
      else {
        this.shoulder = 0.0;
      }
      if (initObj.hasOwnProperty('elbow')) {
        this.elbow = initObj.elbow
      }
      else {
        this.elbow = 0.0;
      }
      if (initObj.hasOwnProperty('gripper')) {
        this.gripper = initObj.gripper
      }
      else {
        this.gripper = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AnglesConverterResponse
    // Serialize message field [base]
    bufferOffset = _serializer.float64(obj.base, buffer, bufferOffset);
    // Serialize message field [shoulder]
    bufferOffset = _serializer.float64(obj.shoulder, buffer, bufferOffset);
    // Serialize message field [elbow]
    bufferOffset = _serializer.float64(obj.elbow, buffer, bufferOffset);
    // Serialize message field [gripper]
    bufferOffset = _serializer.float64(obj.gripper, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AnglesConverterResponse
    let len;
    let data = new AnglesConverterResponse(null);
    // Deserialize message field [base]
    data.base = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [shoulder]
    data.shoulder = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [elbow]
    data.elbow = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gripper]
    data.gripper = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a service object
    return 'arduinobot_controller/AnglesConverterResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '19b04e0ebe735f18fbbd5aea9d85b833';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Response
    float64 base
    float64 shoulder
    float64 elbow
    float64 gripper
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AnglesConverterResponse(null);
    if (msg.base !== undefined) {
      resolved.base = msg.base;
    }
    else {
      resolved.base = 0.0
    }

    if (msg.shoulder !== undefined) {
      resolved.shoulder = msg.shoulder;
    }
    else {
      resolved.shoulder = 0.0
    }

    if (msg.elbow !== undefined) {
      resolved.elbow = msg.elbow;
    }
    else {
      resolved.elbow = 0.0
    }

    if (msg.gripper !== undefined) {
      resolved.gripper = msg.gripper;
    }
    else {
      resolved.gripper = 0.0
    }

    return resolved;
    }
};

module.exports = {
  Request: AnglesConverterRequest,
  Response: AnglesConverterResponse,
  md5sum() { return '904e8346ace63ec441f6f58d8efc6687'; },
  datatype() { return 'arduinobot_controller/AnglesConverter'; }
};
