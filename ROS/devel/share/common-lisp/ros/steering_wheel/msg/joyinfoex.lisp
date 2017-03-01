; Auto-generated. Do not edit!


(cl:in-package steering_wheel-msg)


;//! \htmlinclude joyinfoex.msg.html

(cl:defclass <joyinfoex> (roslisp-msg-protocol:ros-message)
  ((dwXpos
    :reader dwXpos
    :initarg :dwXpos
    :type cl:integer
    :initform 0)
   (dwYpos
    :reader dwYpos
    :initarg :dwYpos
    :type cl:integer
    :initform 0)
   (dwZpos
    :reader dwZpos
    :initarg :dwZpos
    :type cl:integer
    :initform 0)
   (dwRpos
    :reader dwRpos
    :initarg :dwRpos
    :type cl:integer
    :initform 0)
   (dwUpos
    :reader dwUpos
    :initarg :dwUpos
    :type cl:integer
    :initform 0)
   (dwVpos
    :reader dwVpos
    :initarg :dwVpos
    :type cl:integer
    :initform 0)
   (dwButtons
    :reader dwButtons
    :initarg :dwButtons
    :type cl:integer
    :initform 0)
   (dwButtonNumber
    :reader dwButtonNumber
    :initarg :dwButtonNumber
    :type cl:integer
    :initform 0)
   (dwPOV
    :reader dwPOV
    :initarg :dwPOV
    :type cl:integer
    :initform 0))
)

(cl:defclass joyinfoex (<joyinfoex>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <joyinfoex>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'joyinfoex)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name steering_wheel-msg:<joyinfoex> is deprecated: use steering_wheel-msg:joyinfoex instead.")))

(cl:ensure-generic-function 'dwXpos-val :lambda-list '(m))
(cl:defmethod dwXpos-val ((m <joyinfoex>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader steering_wheel-msg:dwXpos-val is deprecated.  Use steering_wheel-msg:dwXpos instead.")
  (dwXpos m))

(cl:ensure-generic-function 'dwYpos-val :lambda-list '(m))
(cl:defmethod dwYpos-val ((m <joyinfoex>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader steering_wheel-msg:dwYpos-val is deprecated.  Use steering_wheel-msg:dwYpos instead.")
  (dwYpos m))

(cl:ensure-generic-function 'dwZpos-val :lambda-list '(m))
(cl:defmethod dwZpos-val ((m <joyinfoex>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader steering_wheel-msg:dwZpos-val is deprecated.  Use steering_wheel-msg:dwZpos instead.")
  (dwZpos m))

(cl:ensure-generic-function 'dwRpos-val :lambda-list '(m))
(cl:defmethod dwRpos-val ((m <joyinfoex>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader steering_wheel-msg:dwRpos-val is deprecated.  Use steering_wheel-msg:dwRpos instead.")
  (dwRpos m))

(cl:ensure-generic-function 'dwUpos-val :lambda-list '(m))
(cl:defmethod dwUpos-val ((m <joyinfoex>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader steering_wheel-msg:dwUpos-val is deprecated.  Use steering_wheel-msg:dwUpos instead.")
  (dwUpos m))

(cl:ensure-generic-function 'dwVpos-val :lambda-list '(m))
(cl:defmethod dwVpos-val ((m <joyinfoex>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader steering_wheel-msg:dwVpos-val is deprecated.  Use steering_wheel-msg:dwVpos instead.")
  (dwVpos m))

(cl:ensure-generic-function 'dwButtons-val :lambda-list '(m))
(cl:defmethod dwButtons-val ((m <joyinfoex>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader steering_wheel-msg:dwButtons-val is deprecated.  Use steering_wheel-msg:dwButtons instead.")
  (dwButtons m))

(cl:ensure-generic-function 'dwButtonNumber-val :lambda-list '(m))
(cl:defmethod dwButtonNumber-val ((m <joyinfoex>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader steering_wheel-msg:dwButtonNumber-val is deprecated.  Use steering_wheel-msg:dwButtonNumber instead.")
  (dwButtonNumber m))

(cl:ensure-generic-function 'dwPOV-val :lambda-list '(m))
(cl:defmethod dwPOV-val ((m <joyinfoex>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader steering_wheel-msg:dwPOV-val is deprecated.  Use steering_wheel-msg:dwPOV instead.")
  (dwPOV m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <joyinfoex>) ostream)
  "Serializes a message object of type '<joyinfoex>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dwXpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dwXpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dwXpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dwXpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dwYpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dwYpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dwYpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dwYpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dwZpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dwZpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dwZpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dwZpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dwRpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dwRpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dwRpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dwRpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dwUpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dwUpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dwUpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dwUpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dwVpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dwVpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dwVpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dwVpos)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dwButtons)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dwButtons)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dwButtons)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dwButtons)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'dwButtons)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'dwButtons)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'dwButtons)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'dwButtons)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dwButtonNumber)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dwButtonNumber)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dwButtonNumber)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dwButtonNumber)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dwPOV)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dwPOV)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dwPOV)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dwPOV)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'dwPOV)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'dwPOV)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'dwPOV)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'dwPOV)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <joyinfoex>) istream)
  "Deserializes a message object of type '<joyinfoex>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dwXpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dwXpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dwXpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dwXpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dwYpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dwYpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dwYpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dwYpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dwZpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dwZpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dwZpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dwZpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dwRpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dwRpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dwRpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dwRpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dwUpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dwUpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dwUpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dwUpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dwVpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dwVpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dwVpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dwVpos)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dwButtons)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dwButtons)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dwButtons)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dwButtons)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'dwButtons)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'dwButtons)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'dwButtons)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'dwButtons)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dwButtonNumber)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dwButtonNumber)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dwButtonNumber)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dwButtonNumber)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'dwPOV)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'dwPOV)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'dwPOV)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'dwPOV)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'dwPOV)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'dwPOV)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'dwPOV)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'dwPOV)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<joyinfoex>)))
  "Returns string type for a message object of type '<joyinfoex>"
  "steering_wheel/joyinfoex")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'joyinfoex)))
  "Returns string type for a message object of type 'joyinfoex"
  "steering_wheel/joyinfoex")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<joyinfoex>)))
  "Returns md5sum for a message object of type '<joyinfoex>"
  "483386359ee4453093908bee904d04de")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'joyinfoex)))
  "Returns md5sum for a message object of type 'joyinfoex"
  "483386359ee4453093908bee904d04de")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<joyinfoex>)))
  "Returns full string definition for message of type '<joyinfoex>"
  (cl:format cl:nil "uint32 dwXpos~%uint32 dwYpos~%uint32 dwZpos~%uint32 dwRpos~%uint32 dwUpos~%uint32 dwVpos~%uint64 dwButtons~%uint32 dwButtonNumber~%uint64 dwPOV~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'joyinfoex)))
  "Returns full string definition for message of type 'joyinfoex"
  (cl:format cl:nil "uint32 dwXpos~%uint32 dwYpos~%uint32 dwZpos~%uint32 dwRpos~%uint32 dwUpos~%uint32 dwVpos~%uint64 dwButtons~%uint32 dwButtonNumber~%uint64 dwPOV~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <joyinfoex>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     8
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <joyinfoex>))
  "Converts a ROS message object to a list"
  (cl:list 'joyinfoex
    (cl:cons ':dwXpos (dwXpos msg))
    (cl:cons ':dwYpos (dwYpos msg))
    (cl:cons ':dwZpos (dwZpos msg))
    (cl:cons ':dwRpos (dwRpos msg))
    (cl:cons ':dwUpos (dwUpos msg))
    (cl:cons ':dwVpos (dwVpos msg))
    (cl:cons ':dwButtons (dwButtons msg))
    (cl:cons ':dwButtonNumber (dwButtonNumber msg))
    (cl:cons ':dwPOV (dwPOV msg))
))
