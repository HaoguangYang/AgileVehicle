// Auto-generated. Do not edit!

// (in-package steering_wheel.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class joyinfoex {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dwXpos = null;
      this.dwYpos = null;
      this.dwZpos = null;
      this.dwRpos = null;
      this.dwUpos = null;
      this.dwVpos = null;
      this.dwButtons = null;
      this.dwButtonNumber = null;
      this.dwPOV = null;
    }
    else {
      if (initObj.hasOwnProperty('dwXpos')) {
        this.dwXpos = initObj.dwXpos
      }
      else {
        this.dwXpos = 0;
      }
      if (initObj.hasOwnProperty('dwYpos')) {
        this.dwYpos = initObj.dwYpos
      }
      else {
        this.dwYpos = 0;
      }
      if (initObj.hasOwnProperty('dwZpos')) {
        this.dwZpos = initObj.dwZpos
      }
      else {
        this.dwZpos = 0;
      }
      if (initObj.hasOwnProperty('dwRpos')) {
        this.dwRpos = initObj.dwRpos
      }
      else {
        this.dwRpos = 0;
      }
      if (initObj.hasOwnProperty('dwUpos')) {
        this.dwUpos = initObj.dwUpos
      }
      else {
        this.dwUpos = 0;
      }
      if (initObj.hasOwnProperty('dwVpos')) {
        this.dwVpos = initObj.dwVpos
      }
      else {
        this.dwVpos = 0;
      }
      if (initObj.hasOwnProperty('dwButtons')) {
        this.dwButtons = initObj.dwButtons
      }
      else {
        this.dwButtons = 0;
      }
      if (initObj.hasOwnProperty('dwButtonNumber')) {
        this.dwButtonNumber = initObj.dwButtonNumber
      }
      else {
        this.dwButtonNumber = 0;
      }
      if (initObj.hasOwnProperty('dwPOV')) {
        this.dwPOV = initObj.dwPOV
      }
      else {
        this.dwPOV = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type joyinfoex
    // Serialize message field [dwXpos]
    bufferOffset = _serializer.uint32(obj.dwXpos, buffer, bufferOffset);
    // Serialize message field [dwYpos]
    bufferOffset = _serializer.uint32(obj.dwYpos, buffer, bufferOffset);
    // Serialize message field [dwZpos]
    bufferOffset = _serializer.uint32(obj.dwZpos, buffer, bufferOffset);
    // Serialize message field [dwRpos]
    bufferOffset = _serializer.uint32(obj.dwRpos, buffer, bufferOffset);
    // Serialize message field [dwUpos]
    bufferOffset = _serializer.uint32(obj.dwUpos, buffer, bufferOffset);
    // Serialize message field [dwVpos]
    bufferOffset = _serializer.uint32(obj.dwVpos, buffer, bufferOffset);
    // Serialize message field [dwButtons]
    bufferOffset = _serializer.uint64(obj.dwButtons, buffer, bufferOffset);
    // Serialize message field [dwButtonNumber]
    bufferOffset = _serializer.uint32(obj.dwButtonNumber, buffer, bufferOffset);
    // Serialize message field [dwPOV]
    bufferOffset = _serializer.uint64(obj.dwPOV, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type joyinfoex
    let len;
    let data = new joyinfoex(null);
    // Deserialize message field [dwXpos]
    data.dwXpos = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [dwYpos]
    data.dwYpos = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [dwZpos]
    data.dwZpos = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [dwRpos]
    data.dwRpos = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [dwUpos]
    data.dwUpos = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [dwVpos]
    data.dwVpos = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [dwButtons]
    data.dwButtons = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [dwButtonNumber]
    data.dwButtonNumber = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [dwPOV]
    data.dwPOV = _deserializer.uint64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'steering_wheel/joyinfoex';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '483386359ee4453093908bee904d04de';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 dwXpos
    uint32 dwYpos
    uint32 dwZpos
    uint32 dwRpos
    uint32 dwUpos
    uint32 dwVpos
    uint64 dwButtons
    uint32 dwButtonNumber
    uint64 dwPOV
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new joyinfoex(null);
    if (msg.dwXpos !== undefined) {
      resolved.dwXpos = msg.dwXpos;
    }
    else {
      resolved.dwXpos = 0
    }

    if (msg.dwYpos !== undefined) {
      resolved.dwYpos = msg.dwYpos;
    }
    else {
      resolved.dwYpos = 0
    }

    if (msg.dwZpos !== undefined) {
      resolved.dwZpos = msg.dwZpos;
    }
    else {
      resolved.dwZpos = 0
    }

    if (msg.dwRpos !== undefined) {
      resolved.dwRpos = msg.dwRpos;
    }
    else {
      resolved.dwRpos = 0
    }

    if (msg.dwUpos !== undefined) {
      resolved.dwUpos = msg.dwUpos;
    }
    else {
      resolved.dwUpos = 0
    }

    if (msg.dwVpos !== undefined) {
      resolved.dwVpos = msg.dwVpos;
    }
    else {
      resolved.dwVpos = 0
    }

    if (msg.dwButtons !== undefined) {
      resolved.dwButtons = msg.dwButtons;
    }
    else {
      resolved.dwButtons = 0
    }

    if (msg.dwButtonNumber !== undefined) {
      resolved.dwButtonNumber = msg.dwButtonNumber;
    }
    else {
      resolved.dwButtonNumber = 0
    }

    if (msg.dwPOV !== undefined) {
      resolved.dwPOV = msg.dwPOV;
    }
    else {
      resolved.dwPOV = 0
    }

    return resolved;
    }
};

module.exports = joyinfoex;
