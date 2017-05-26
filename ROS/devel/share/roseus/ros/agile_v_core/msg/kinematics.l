;; Auto-generated. Do not edit!


(when (boundp 'agile_v_core::kinematics)
  (if (not (find-package "AGILE_V_CORE"))
    (make-package "AGILE_V_CORE"))
  (shadow 'kinematics (find-package "AGILE_V_CORE")))
(unless (find-package "AGILE_V_CORE::KINEMATICS")
  (make-package "AGILE_V_CORE::KINEMATICS"))

(in-package "ROS")
;;//! \htmlinclude kinematics.msg.html


(defclass agile_v_core::kinematics
  :super ros::object
  :slots (_CM_Velocity _CM_Acceleration _Vehicle_Heading _CM_AngularVel _CM_AngularAcc _Wheel_LinearVel _Wheel_SteerAngl _Time_Stamp ))

(defmethod agile_v_core::kinematics
  (:init
   (&key
    ((:CM_Velocity __CM_Velocity) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:CM_Acceleration __CM_Acceleration) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:Vehicle_Heading __Vehicle_Heading) 0.0)
    ((:CM_AngularVel __CM_AngularVel) 0.0)
    ((:CM_AngularAcc __CM_AngularAcc) 0.0)
    ((:Wheel_LinearVel __Wheel_LinearVel) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:Wheel_SteerAngl __Wheel_SteerAngl) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:Time_Stamp __Time_Stamp) (instance ros::time :init))
    )
   (send-super :init)
   (setq _CM_Velocity __CM_Velocity)
   (setq _CM_Acceleration __CM_Acceleration)
   (setq _Vehicle_Heading (float __Vehicle_Heading))
   (setq _CM_AngularVel (float __CM_AngularVel))
   (setq _CM_AngularAcc (float __CM_AngularAcc))
   (setq _Wheel_LinearVel __Wheel_LinearVel)
   (setq _Wheel_SteerAngl __Wheel_SteerAngl)
   (setq _Time_Stamp __Time_Stamp)
   self)
  (:CM_Velocity
   (&optional __CM_Velocity)
   (if __CM_Velocity (setq _CM_Velocity __CM_Velocity)) _CM_Velocity)
  (:CM_Acceleration
   (&optional __CM_Acceleration)
   (if __CM_Acceleration (setq _CM_Acceleration __CM_Acceleration)) _CM_Acceleration)
  (:Vehicle_Heading
   (&optional __Vehicle_Heading)
   (if __Vehicle_Heading (setq _Vehicle_Heading __Vehicle_Heading)) _Vehicle_Heading)
  (:CM_AngularVel
   (&optional __CM_AngularVel)
   (if __CM_AngularVel (setq _CM_AngularVel __CM_AngularVel)) _CM_AngularVel)
  (:CM_AngularAcc
   (&optional __CM_AngularAcc)
   (if __CM_AngularAcc (setq _CM_AngularAcc __CM_AngularAcc)) _CM_AngularAcc)
  (:Wheel_LinearVel
   (&optional __Wheel_LinearVel)
   (if __Wheel_LinearVel (setq _Wheel_LinearVel __Wheel_LinearVel)) _Wheel_LinearVel)
  (:Wheel_SteerAngl
   (&optional __Wheel_SteerAngl)
   (if __Wheel_SteerAngl (setq _Wheel_SteerAngl __Wheel_SteerAngl)) _Wheel_SteerAngl)
  (:Time_Stamp
   (&optional __Time_Stamp)
   (if __Time_Stamp (setq _Time_Stamp __Time_Stamp)) _Time_Stamp)
  (:serialization-length
   ()
   (+
    ;; float64[2] _CM_Velocity
    (* 8    2)
    ;; float64[2] _CM_Acceleration
    (* 8    2)
    ;; float64 _Vehicle_Heading
    8
    ;; float64 _CM_AngularVel
    8
    ;; float64 _CM_AngularAcc
    8
    ;; float64[4] _Wheel_LinearVel
    (* 8    4)
    ;; float64[4] _Wheel_SteerAngl
    (* 8    4)
    ;; time _Time_Stamp
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[2] _CM_Velocity
     (dotimes (i 2)
       (sys::poke (elt _CM_Velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[2] _CM_Acceleration
     (dotimes (i 2)
       (sys::poke (elt _CM_Acceleration i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _Vehicle_Heading
       (sys::poke _Vehicle_Heading (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _CM_AngularVel
       (sys::poke _CM_AngularVel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _CM_AngularAcc
       (sys::poke _CM_AngularAcc (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[4] _Wheel_LinearVel
     (dotimes (i 4)
       (sys::poke (elt _Wheel_LinearVel i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[4] _Wheel_SteerAngl
     (dotimes (i 4)
       (sys::poke (elt _Wheel_SteerAngl i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; time _Time_Stamp
       (write-long (send _Time_Stamp :sec) s) (write-long (send _Time_Stamp :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[2] _CM_Velocity
   (dotimes (i (length _CM_Velocity))
     (setf (elt _CM_Velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[2] _CM_Acceleration
   (dotimes (i (length _CM_Acceleration))
     (setf (elt _CM_Acceleration i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _Vehicle_Heading
     (setq _Vehicle_Heading (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _CM_AngularVel
     (setq _CM_AngularVel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _CM_AngularAcc
     (setq _CM_AngularAcc (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[4] _Wheel_LinearVel
   (dotimes (i (length _Wheel_LinearVel))
     (setf (elt _Wheel_LinearVel i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[4] _Wheel_SteerAngl
   (dotimes (i (length _Wheel_SteerAngl))
     (setf (elt _Wheel_SteerAngl i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; time _Time_Stamp
     (send _Time_Stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _Time_Stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get agile_v_core::kinematics :md5sum-) "106ad333403d5f1ecfca19cf013ec05f")
(setf (get agile_v_core::kinematics :datatype-) "agile_v_core/kinematics")
(setf (get agile_v_core::kinematics :definition-)
      "# Vehicle Center Kinematics on Vehicle Coordinates, MKS Units Mandatory.
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

")



(provide :agile_v_core/kinematics "106ad333403d5f1ecfca19cf013ec05f")


