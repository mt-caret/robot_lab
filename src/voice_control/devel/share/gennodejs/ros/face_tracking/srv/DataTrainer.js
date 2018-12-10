// Auto-generated. Do not edit!

// (in-package face_tracking.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class DataTrainerRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.onoff = null;
    }
    else {
      if (initObj.hasOwnProperty('onoff')) {
        this.onoff = initObj.onoff
      }
      else {
        this.onoff = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DataTrainerRequest
    // Serialize message field [onoff]
    bufferOffset = _serializer.bool(obj.onoff, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DataTrainerRequest
    let len;
    let data = new DataTrainerRequest(null);
    // Deserialize message field [onoff]
    data.onoff = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'face_tracking/DataTrainerRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8e9546cddc70e33b0bccfc5125b3a46d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool onoff
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DataTrainerRequest(null);
    if (msg.onoff !== undefined) {
      resolved.onoff = msg.onoff;
    }
    else {
      resolved.onoff = false
    }

    return resolved;
    }
};

class DataTrainerResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.num = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('num')) {
        this.num = initObj.num
      }
      else {
        this.num = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DataTrainerResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [num]
    bufferOffset = _serializer.int64(obj.num, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DataTrainerResponse
    let len;
    let data = new DataTrainerResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [num]
    data.num = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'face_tracking/DataTrainerResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '78ba3fd2726937dea1600a73af3b8787';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    int64 num
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DataTrainerResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.num !== undefined) {
      resolved.num = msg.num;
    }
    else {
      resolved.num = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: DataTrainerRequest,
  Response: DataTrainerResponse,
  md5sum() { return '07f48dc3148a93e6660a9d3c75b3c6af'; },
  datatype() { return 'face_tracking/DataTrainer'; }
};
