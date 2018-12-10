// Auto-generated. Do not edit!

// (in-package face_tracking.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Dist {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.anglar = null;
      this.dist = null;
    }
    else {
      if (initObj.hasOwnProperty('anglar')) {
        this.anglar = initObj.anglar
      }
      else {
        this.anglar = 0.0;
      }
      if (initObj.hasOwnProperty('dist')) {
        this.dist = initObj.dist
      }
      else {
        this.dist = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Dist
    // Serialize message field [anglar]
    bufferOffset = _serializer.float32(obj.anglar, buffer, bufferOffset);
    // Serialize message field [dist]
    bufferOffset = _serializer.int32(obj.dist, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Dist
    let len;
    let data = new Dist(null);
    // Deserialize message field [anglar]
    data.anglar = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dist]
    data.dist = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'face_tracking/Dist';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2217c177b412cc5ac6a43674c38fcf6a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 anglar
    int32 dist
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Dist(null);
    if (msg.anglar !== undefined) {
      resolved.anglar = msg.anglar;
    }
    else {
      resolved.anglar = 0.0
    }

    if (msg.dist !== undefined) {
      resolved.dist = msg.dist;
    }
    else {
      resolved.dist = 0
    }

    return resolved;
    }
};

module.exports = Dist;
