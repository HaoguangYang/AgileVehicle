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

class kinematics {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.CM_Velocity = null;
      this.CM_Acceleration = null;
      this.Vehicle_Heading = null;
      this.CM_AngularVel = null;
      this.CM_AngularAcc = null;
      this.Wheel_LinearVel = null;
      this.Wheel_SteerAngl = null;
      this.Time_Stamp = null;
    }
    else {
      if (initObj.hasOwnProperty('CM_Velocity')) {
        this.CM_Velocity = initObj.CM_Velocity
      }
      else {
        this.CM_Velocity = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('CM_Acceleration')) {
        this.CM_Acceleration = initObj.CM_Acceleration
      }
      else {
        this.CM_Acceleration = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('Vehicle_Heading')) {
        this.Vehicle_Heading = initObj.Vehicle_Heading
      }
      else {
        this.Vehicle_Heading = 0.0;
      }
      if (initObj.hasOwnProperty('CM_AngularVel')) {
        this.CM_AngularVel = initObj.CM_AngularVel
      }
      else {
        this.CM_AngularVel = 0.0;
      }
      if (initObj.hasOwnProperty('CM_AngularAcc')) {
        this.CM_AngularAcc = initObj.CM_AngularAcc
      }
      else {
        this.CM_AngularAcc = 0.0;
      }
      if (initObj.hasOwnProperty('Wheel_LinearVel')) {
        this.Wheel_LinearVel = initObj.Wheel_LinearVel
      }
      else {
        this.Wheel_LinearVel = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('Wheel_SteerAngl')) {
        this.Wheel_SteerAngl = initObj.Wheel_SteerAngl
      }
      else {
        this.Wheel_SteerAngl = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('Time_Stamp')) {
        this.Time_Stamp = initObj.Time_Stamp
      }
      else {
        this.Time_Stamp = {secs: 0, nsecs: 0};
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type kinematics
    // Check that the constant length array field [CM_Velocity] has the right length
    if (obj.CM_Velocity.length !== 2) {
      throw new Error('Unable to serialize array field CM_Velocity - length must be 2')
    }
    // Serialize message field [CM_Velocity]
    bufferOffset = _arraySerializer.float64(obj.CM_Velocity, buffer, bufferOffset, 2);
    // Check that the constant length array field [CM_Acceleration] has the right length
    if (obj.CM_Acceleration.length !== 2) {
      throw new Error('Unable to serialize array field CM_Acceleration - length must be 2')
    }
    // Serialize message field [CM_Acceleration]
    bufferOffset = _arraySerializer.float64(obj.CM_Acceleration, buffer, bufferOffset, 2);
    // Serialize message field [Vehicle_Heading]
    bufferOffset = _serializer.float64(obj.Vehicle_Heading, buffer, bufferOffset);
    // Serialize message field [CM_AngularVel]
    bufferOffset = _serializer.float64(obj.CM_AngularVel, buffer, bufferOffset);
    // Serialize message field [CM_AngularAcc]
    bufferOffset = _serializer.float64(obj.CM_AngularAcc, buffer, bufferOffset);
    // Check that the constant length array field [Wheel_LinearVel] has the right length
    if (obj.Wheel_LinearVel.length !== 4) {
      throw new Error('Unable to serialize array field Wheel_LinearVel - length must be 4')
    }
    // Serialize message field [Wheel_LinearVel]
    bufferOffset = _arraySerializer.float64(obj.Wheel_LinearVel, buffer, bufferOffset, 4);
    // Check that the constant length array field [Wheel_SteerAngl] has the right length
    if (obj.Wheel_SteerAngl.length !== 4) {
      throw new Error('Unable to serialize array field Wheel_SteerAngl - length must be 4')
    }
    // Serialize message field [Wheel_SteerAngl]
    bufferOffset = _arraySerializer.float64(obj.Wheel_SteerAngl, buffer, bufferOffset, 4);
    // Serialize message field [Time_Stamp]
    bufferOffset = _serializer.time(obj.Time_Stamp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type kinematics
    let len;
    let data = new kinematics(null);
    // Deserialize message field [CM_Velocity]
    data.CM_Velocity = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    // Deserialize message field [CM_Acceleration]
    data.CM_Acceleration = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    // Deserialize message field [Vehicle_Heading]
    data.Vehicle_Heading = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [CM_AngularVel]
    data.CM_AngularVel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [CM_AngularAcc]
    data.CM_AngularAcc = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Wheel_LinearVel]
    data.Wheel_LinearVel = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    // Deserialize message field [Wheel_SteerAngl]
    data.Wheel_SteerAngl = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    // Deserialize message field [Time_Stamp]
    data.Time_Stamp = _deserializer.time(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 128;
  }

  static datatype() {
    // Returns string type for a message object
    return 'agile_v_core/kinematics';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '106ad333403d5f1ecfca19cf013ec05f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Vehicle Center Kinematics on Vehicle Coordinates, MKS Units Mandatory.
    float64[2]   CM_Velocity
    float64[2]   CM_Acceleration
    float64     Vehicle_Heading
    float64     CM_AngularVel
    float64     CM_AngularAcc
    
    # Wheel Kinematics
    float64[4]   Wheel_LinearVel
    float64[4]   Wheel_SteerAngl
    
    # Time Stamp
    time        Time_Stamp
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new kinematics(null);
    if (msg.CM_Velocity !== undefined) {
      resolved.CM_Velocity = msg.CM_Velocity;
    }
    else {
      resolved.CM_Velocity = new Array(2).fill(0)
    }

    if (msg.CM_Acceleration !== undefined) {
      resolved.CM_Acceleration = msg.CM_Acceleration;
    }
    else {
      resolved.CM_Acceleration = new Array(2).fill(0)
    }

    if (msg.Vehicle_Heading !== undefined) {
      resolved.Vehicle_Heading = msg.Vehicle_Heading;
    }
    else {
      resolved.Vehicle_Heading = 0.0
    }

    if (msg.CM_AngularVel !== undefined) {
      resolved.CM_AngularVel = msg.CM_AngularVel;
    }
    else {
      resolved.CM_AngularVel = 0.0
    }

    if (msg.CM_AngularAcc !== undefined) {
      resolved.CM_AngularAcc = msg.CM_AngularAcc;
    }
    else {
      resolved.CM_AngularAcc = 0.0
    }

    if (msg.Wheel_LinearVel !== undefined) {
      resolved.Wheel_LinearVel = msg.Wheel_LinearVel;
    }
    else {
      resolved.Wheel_LinearVel = new Array(4).fill(0)
    }

    if (msg.Wheel_SteerAngl !== undefined) {
      resolved.Wheel_SteerAngl = msg.Wheel_SteerAngl;
    }
    else {
      resolved.Wheel_SteerAngl = new Array(4).fill(0)
    }

    if (msg.Time_Stamp !== undefined) {
      resolved.Time_Stamp = msg.Time_Stamp;
    }
    else {
      resolved.Time_Stamp = {secs: 0, nsecs: 0}
    }

    return resolved;
    }
};

module.exports = kinematics;
