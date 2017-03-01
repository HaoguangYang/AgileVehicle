// Auto-generated. Do not edit!

// (in-package steering_wheel.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class joyinfoex {
  constructor() {
    this.dwXpos = 0;
    this.dwYpos = 0;
    this.dwZpos = 0;
    this.dwRpos = 0;
    this.dwUpos = 0;
    this.dwVpos = 0;
    this.dwButtons = 0;
    this.dwButtonNumber = 0;
    this.dwPOV = 0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type joyinfoex
    // Serialize message field [dwXpos]
    bufferInfo = _serializer.uint32(obj.dwXpos, bufferInfo);
    // Serialize message field [dwYpos]
    bufferInfo = _serializer.uint32(obj.dwYpos, bufferInfo);
    // Serialize message field [dwZpos]
    bufferInfo = _serializer.uint32(obj.dwZpos, bufferInfo);
    // Serialize message field [dwRpos]
    bufferInfo = _serializer.uint32(obj.dwRpos, bufferInfo);
    // Serialize message field [dwUpos]
    bufferInfo = _serializer.uint32(obj.dwUpos, bufferInfo);
    // Serialize message field [dwVpos]
    bufferInfo = _serializer.uint32(obj.dwVpos, bufferInfo);
    // Serialize message field [dwButtons]
    bufferInfo = _serializer.uint64(obj.dwButtons, bufferInfo);
    // Serialize message field [dwButtonNumber]
    bufferInfo = _serializer.uint32(obj.dwButtonNumber, bufferInfo);
    // Serialize message field [dwPOV]
    bufferInfo = _serializer.uint64(obj.dwPOV, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type joyinfoex
    let tmp;
    let len;
    let data = new joyinfoex();
    // Deserialize message field [dwXpos]
    tmp = _deserializer.uint32(buffer);
    data.dwXpos = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [dwYpos]
    tmp = _deserializer.uint32(buffer);
    data.dwYpos = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [dwZpos]
    tmp = _deserializer.uint32(buffer);
    data.dwZpos = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [dwRpos]
    tmp = _deserializer.uint32(buffer);
    data.dwRpos = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [dwUpos]
    tmp = _deserializer.uint32(buffer);
    data.dwUpos = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [dwVpos]
    tmp = _deserializer.uint32(buffer);
    data.dwVpos = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [dwButtons]
    tmp = _deserializer.uint64(buffer);
    data.dwButtons = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [dwButtonNumber]
    tmp = _deserializer.uint32(buffer);
    data.dwButtonNumber = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [dwPOV]
    tmp = _deserializer.uint64(buffer);
    data.dwPOV = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
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

};

module.exports = joyinfoex;
