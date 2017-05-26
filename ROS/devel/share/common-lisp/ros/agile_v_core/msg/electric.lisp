; Auto-generated. Do not edit!


(cl:in-package agile_v_core-msg)


;//! \htmlinclude electric.msg.html

(cl:defclass <electric> (roslisp-msg-protocol:ros-message)
  ((voltage
    :reader voltage
    :initarg :voltage
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (pwrDriving
    :reader pwrDriving
    :initarg :pwrDriving
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (pwrSteering
    :reader pwrSteering
    :initarg :pwrSteering
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (pwrTotal
    :reader pwrTotal
    :initarg :pwrTotal
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (AmpDriving
    :reader AmpDriving
    :initarg :AmpDriving
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (AmpSteering
    :reader AmpSteering
    :initarg :AmpSteering
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (UnitAmp
    :reader UnitAmp
    :initarg :UnitAmp
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass electric (<electric>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <electric>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'electric)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name agile_v_core-msg:<electric> is deprecated: use agile_v_core-msg:electric instead.")))

(cl:ensure-generic-function 'voltage-val :lambda-list '(m))
(cl:defmethod voltage-val ((m <electric>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agile_v_core-msg:voltage-val is deprecated.  Use agile_v_core-msg:voltage instead.")
  (voltage m))

(cl:ensure-generic-function 'pwrDriving-val :lambda-list '(m))
(cl:defmethod pwrDriving-val ((m <electric>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agile_v_core-msg:pwrDriving-val is deprecated.  Use agile_v_core-msg:pwrDriving instead.")
  (pwrDriving m))

(cl:ensure-generic-function 'pwrSteering-val :lambda-list '(m))
(cl:defmethod pwrSteering-val ((m <electric>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agile_v_core-msg:pwrSteering-val is deprecated.  Use agile_v_core-msg:pwrSteering instead.")
  (pwrSteering m))

(cl:ensure-generic-function 'pwrTotal-val :lambda-list '(m))
(cl:defmethod pwrTotal-val ((m <electric>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agile_v_core-msg:pwrTotal-val is deprecated.  Use agile_v_core-msg:pwrTotal instead.")
  (pwrTotal m))

(cl:ensure-generic-function 'AmpDriving-val :lambda-list '(m))
(cl:defmethod AmpDriving-val ((m <electric>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agile_v_core-msg:AmpDriving-val is deprecated.  Use agile_v_core-msg:AmpDriving instead.")
  (AmpDriving m))

(cl:ensure-generic-function 'AmpSteering-val :lambda-list '(m))
(cl:defmethod AmpSteering-val ((m <electric>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agile_v_core-msg:AmpSteering-val is deprecated.  Use agile_v_core-msg:AmpSteering instead.")
  (AmpSteering m))

(cl:ensure-generic-function 'UnitAmp-val :lambda-list '(m))
(cl:defmethod UnitAmp-val ((m <electric>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader agile_v_core-msg:UnitAmp-val is deprecated.  Use agile_v_core-msg:UnitAmp instead.")
  (UnitAmp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <electric>) ostream)
  "Serializes a message object of type '<electric>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'voltage))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'pwrDriving))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'pwrSteering))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'pwrTotal))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'AmpDriving))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'AmpSteering))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'UnitAmp))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <electric>) istream)
  "Deserializes a message object of type '<electric>"
  (cl:setf (cl:slot-value msg 'voltage) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'voltage)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'pwrDriving) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'pwrDriving)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'pwrSteering) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'pwrSteering)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'pwrTotal) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'pwrTotal)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'AmpDriving) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'AmpDriving)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'AmpSteering) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'AmpSteering)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'UnitAmp) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'UnitAmp)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<electric>)))
  "Returns string type for a message object of type '<electric>"
  "agile_v_core/electric")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'electric)))
  "Returns string type for a message object of type 'electric"
  "agile_v_core/electric")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<electric>)))
  "Returns md5sum for a message object of type '<electric>"
  "c6bc6c3c8d1a9c04520cae2eed8a6f9d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'electric)))
  "Returns md5sum for a message object of type 'electric"
  "c6bc6c3c8d1a9c04520cae2eed8a6f9d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<electric>)))
  "Returns full string definition for message of type '<electric>"
  (cl:format cl:nil "float32[4] voltage~%float32[4] pwrDriving~%float32[4] pwrSteering~%float32[4] pwrTotal~%float32[4] AmpDriving~%float32[4] AmpSteering~%float32[4] UnitAmp~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'electric)))
  "Returns full string definition for message of type 'electric"
  (cl:format cl:nil "float32[4] voltage~%float32[4] pwrDriving~%float32[4] pwrSteering~%float32[4] pwrTotal~%float32[4] AmpDriving~%float32[4] AmpSteering~%float32[4] UnitAmp~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <electric>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'voltage) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pwrDriving) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pwrSteering) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pwrTotal) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'AmpDriving) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'AmpSteering) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'UnitAmp) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <electric>))
  "Converts a ROS message object to a list"
  (cl:list 'electric
    (cl:cons ':voltage (voltage msg))
    (cl:cons ':pwrDriving (pwrDriving msg))
    (cl:cons ':pwrSteering (pwrSteering msg))
    (cl:cons ':pwrTotal (pwrTotal msg))
    (cl:cons ':AmpDriving (AmpDriving msg))
    (cl:cons ':AmpSteering (AmpSteering msg))
    (cl:cons ':UnitAmp (UnitAmp msg))
))
