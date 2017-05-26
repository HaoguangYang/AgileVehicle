;; Auto-generated. Do not edit!


(when (boundp 'agile_v_core::electric)
  (if (not (find-package "AGILE_V_CORE"))
    (make-package "AGILE_V_CORE"))
  (shadow 'electric (find-package "AGILE_V_CORE")))
(unless (find-package "AGILE_V_CORE::ELECTRIC")
  (make-package "AGILE_V_CORE::ELECTRIC"))

(in-package "ROS")
;;//! \htmlinclude electric.msg.html


(defclass agile_v_core::electric
  :super ros::object
  :slots (_voltage _pwrDriving _pwrSteering _pwrTotal _AmpDriving _AmpSteering _UnitAmp ))

(defmethod agile_v_core::electric
  (:init
   (&key
    ((:voltage __voltage) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:pwrDriving __pwrDriving) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:pwrSteering __pwrSteering) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:pwrTotal __pwrTotal) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:AmpDriving __AmpDriving) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:AmpSteering __AmpSteering) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:UnitAmp __UnitAmp) (make-array 4 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _voltage __voltage)
   (setq _pwrDriving __pwrDriving)
   (setq _pwrSteering __pwrSteering)
   (setq _pwrTotal __pwrTotal)
   (setq _AmpDriving __AmpDriving)
   (setq _AmpSteering __AmpSteering)
   (setq _UnitAmp __UnitAmp)
   self)
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:pwrDriving
   (&optional __pwrDriving)
   (if __pwrDriving (setq _pwrDriving __pwrDriving)) _pwrDriving)
  (:pwrSteering
   (&optional __pwrSteering)
   (if __pwrSteering (setq _pwrSteering __pwrSteering)) _pwrSteering)
  (:pwrTotal
   (&optional __pwrTotal)
   (if __pwrTotal (setq _pwrTotal __pwrTotal)) _pwrTotal)
  (:AmpDriving
   (&optional __AmpDriving)
   (if __AmpDriving (setq _AmpDriving __AmpDriving)) _AmpDriving)
  (:AmpSteering
   (&optional __AmpSteering)
   (if __AmpSteering (setq _AmpSteering __AmpSteering)) _AmpSteering)
  (:UnitAmp
   (&optional __UnitAmp)
   (if __UnitAmp (setq _UnitAmp __UnitAmp)) _UnitAmp)
  (:serialization-length
   ()
   (+
    ;; float32[4] _voltage
    (* 4    4)
    ;; float32[4] _pwrDriving
    (* 4    4)
    ;; float32[4] _pwrSteering
    (* 4    4)
    ;; float32[4] _pwrTotal
    (* 4    4)
    ;; float32[4] _AmpDriving
    (* 4    4)
    ;; float32[4] _AmpSteering
    (* 4    4)
    ;; float32[4] _UnitAmp
    (* 4    4)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[4] _voltage
     (dotimes (i 4)
       (sys::poke (elt _voltage i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[4] _pwrDriving
     (dotimes (i 4)
       (sys::poke (elt _pwrDriving i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[4] _pwrSteering
     (dotimes (i 4)
       (sys::poke (elt _pwrSteering i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[4] _pwrTotal
     (dotimes (i 4)
       (sys::poke (elt _pwrTotal i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[4] _AmpDriving
     (dotimes (i 4)
       (sys::poke (elt _AmpDriving i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[4] _AmpSteering
     (dotimes (i 4)
       (sys::poke (elt _AmpSteering i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[4] _UnitAmp
     (dotimes (i 4)
       (sys::poke (elt _UnitAmp i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[4] _voltage
   (dotimes (i (length _voltage))
     (setf (elt _voltage i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[4] _pwrDriving
   (dotimes (i (length _pwrDriving))
     (setf (elt _pwrDriving i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[4] _pwrSteering
   (dotimes (i (length _pwrSteering))
     (setf (elt _pwrSteering i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[4] _pwrTotal
   (dotimes (i (length _pwrTotal))
     (setf (elt _pwrTotal i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[4] _AmpDriving
   (dotimes (i (length _AmpDriving))
     (setf (elt _AmpDriving i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[4] _AmpSteering
   (dotimes (i (length _AmpSteering))
     (setf (elt _AmpSteering i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[4] _UnitAmp
   (dotimes (i (length _UnitAmp))
     (setf (elt _UnitAmp i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get agile_v_core::electric :md5sum-) "c6bc6c3c8d1a9c04520cae2eed8a6f9d")
(setf (get agile_v_core::electric :datatype-) "agile_v_core/electric")
(setf (get agile_v_core::electric :definition-)
      "float32[4] voltage
float32[4] pwrDriving
float32[4] pwrSteering
float32[4] pwrTotal
float32[4] AmpDriving
float32[4] AmpSteering
float32[4] UnitAmp


")



(provide :agile_v_core/electric "c6bc6c3c8d1a9c04520cae2eed8a6f9d")


