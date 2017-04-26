// Auto-generated. Do not edit!

// (in-package agile_v_core.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class electric {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.voltage = null;
      this.pwrDriving = null;
      this.pwrSteering = null;
      this.pwrTotal = null;
      this.AmpDriving = null;
      this.AmpSteering = null;
      this.UnitAmp = null;
    }
    else {
      if (initObj.hasOwnProperty('voltage')) {
        this.voltage = initObj.voltage
      }
      else {
        this.voltage = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('pwrDriving')) {
        this.pwrDriving = initObj.pwrDriving
      }
      else {
        this.pwrDriving = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('pwrSteering')) {
        this.pwrSteering = initObj.pwrSteering
      }
      else {
        this.pwrSteering = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('pwrTotal')) {
        this.pwrTotal = initObj.pwrTotal
      }
      else {
        this.pwrTotal = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('AmpDriving')) {
        this.AmpDriving = initObj.AmpDriving
      }
      else {
        this.AmpDriving = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('AmpSteering')) {
        this.AmpSteering = initObj.AmpSteering
      }
      else {
        this.AmpSteering = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('UnitAmp')) {
        this.UnitAmp = initObj.UnitAmp
      }
      else {
        this.UnitAmp = new Array(4).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type electric
    // Check that the constant length array field [voltage] has the right length
    if (obj.voltage.length !== 4) {
      throw new Error('Unable to serialize array field voltage - length must be 4')
    }
    // Serialize message field [voltage]
    bufferOffset = _arraySerializer.float32(obj.voltage, buffer, bufferOffset, 4);
    // Check that the constant length array field [pwrDriving] has the right length
    if (obj.pwrDriving.length !== 4) {
      throw new Error('Unable to serialize array field pwrDriving - length must be 4')
    }
    // Serialize message field [pwrDriving]
    bufferOffset = _arraySerializer.float32(obj.pwrDriving, buffer, bufferOffset, 4);
    // Check that the constant length array field [pwrSteering] has the right length
    if (obj.pwrSteering.length !== 4) {
      throw new Error('Unable to serialize array field pwrSteering - length must be 4')
    }
    // Serialize message field [pwrSteering]
    bufferOffset = _arraySerializer.float32(obj.pwrSteering, buffer, bufferOffset, 4);
    // Check that the constant length array field [pwrTotal] has the right length
    if (obj.pwrTotal.length !== 4) {
      throw new Error('Unable to serialize array field pwrTotal - length must be 4')
    }
    // Serialize message field [pwrTotal]
    bufferOffset = _arraySerializer.float32(obj.pwrTotal, buffer, bufferOffset, 4);
    // Check that the constant length array field [AmpDriving] has the right length
    if (obj.AmpDriving.length !== 4) {
      throw new Error('Unable to serialize array field AmpDriving - length must be 4')
    }
    // Serialize message field [AmpDriving]
    bufferOffset = _arraySerializer.float32(obj.AmpDriving, buffer, bufferOffset, 4);
    // Check that the constant length array field [AmpSteering] has the right length
    if (obj.AmpSteering.length !== 4) {
      throw new Error('Unable to serialize array field AmpSteering - length must be 4')
    }
    // Serialize message field [AmpSteering]
    bufferOffset = _arraySerializer.float32(obj.AmpSteering, buffer, bufferOffset, 4);
    // Check that the constant length array field [UnitAmp] has the right length
    if (obj.UnitAmp.length !== 4) {
      throw new Error('Unable to serialize array field UnitAmp - length must be 4')
    }
    // Serialize message field [UnitAmp]
    bufferOffset = _arraySerializer.float32(obj.UnitAmp, buffer, bufferOffset, 4);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type electric
    let len;
    let data = new electric(null);
    // Deserialize message field [voltage]
    data.voltage = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    // Deserialize message field [pwrDriving]
    data.pwrDriving = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    // Deserialize message field [pwrSteering]
    data.pwrSteering = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    // Deserialize message field [pwrTotal]
    data.pwrTotal = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    // Deserialize message field [AmpDriving]
    data.AmpDriving = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    // Deserialize message field [AmpSteering]
    data.AmpSteering = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    // Deserialize message field [UnitAmp]
    data.UnitAmp = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    return data;
  }

  static getMessageSize(object) {
    return 112;
  }

  static datatype() {
    // Returns string type for a message object
    return 'agile_v_core/electric';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c6bc6c3c8d1a9c04520cae2eed8a6f9d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[4] voltage
    float32[4] pwrDriving
    float32[4] pwrSteering
    float32[4] pwrTotal
    float32[4] AmpDriving
    float32[4] AmpSteering
    float32[4] UnitAmp
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new electric(null);
    if (msg.voltage !== undefined) {
      resolved.voltage = msg.voltage;
    }
    else {
      resolved.voltage = new Array(4).fill(0)
    }

    if (msg.pwrDriving !== undefined) {
      resolved.pwrDriving = msg.pwrDriving;
    }
    else {
      resolved.pwrDriving = new Array(4).fill(0)
    }

    if (msg.pwrSteering !== undefined) {
      resolved.pwrSteering = msg.pwrSteering;
    }
    else {
      resolved.pwrSteering = new Array(4).fill(0)
    }

    if (msg.pwrTotal !== undefined) {
      resolved.pwrTotal = msg.pwrTotal;
    }
    else {
      resolved.pwrTotal = new Array(4).fill(0)
    }

    if (msg.AmpDriving !== undefined) {
      resolved.AmpDriving = msg.AmpDriving;
    }
    else {
      resolved.AmpDriving = new Array(4).fill(0)
    }

    if (msg.AmpSteering !== undefined) {
      resolved.AmpSteering = msg.AmpSteering;
    }
    else {
      resolved.AmpSteering = new Array(4).fill(0)
    }

    if (msg.UnitAmp !== undefined) {
      resolved.UnitAmp = msg.UnitAmp;
    }
    else {
      resolved.UnitAmp = new Array(4).fill(0)
    }

    return resolved;
    }
};

module.exports = electric;
