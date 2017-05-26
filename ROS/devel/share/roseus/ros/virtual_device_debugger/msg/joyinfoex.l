;; Auto-generated. Do not edit!


(when (boundp 'virtual_device_debugger::joyinfoex)
  (if (not (find-package "VIRTUAL_DEVICE_DEBUGGER"))
    (make-package "VIRTUAL_DEVICE_DEBUGGER"))
  (shadow 'joyinfoex (find-package "VIRTUAL_DEVICE_DEBUGGER")))
(unless (find-package "VIRTUAL_DEVICE_DEBUGGER::JOYINFOEX")
  (make-package "VIRTUAL_DEVICE_DEBUGGER::JOYINFOEX"))

(in-package "ROS")
;;//! \htmlinclude joyinfoex.msg.html


(defclass virtual_device_debugger::joyinfoex
  :super ros::object
  :slots (_dwXpos _dwYpos _dwZpos _dwRpos _dwUpos _dwVpos _dwButtons _dwButtonNumber _dwPOV ))

(defmethod virtual_device_debugger::joyinfoex
  (:init
   (&key
    ((:dwXpos __dwXpos) 0)
    ((:dwYpos __dwYpos) 0)
    ((:dwZpos __dwZpos) 0)
    ((:dwRpos __dwRpos) 0)
    ((:dwUpos __dwUpos) 0)
    ((:dwVpos __dwVpos) 0)
    ((:dwButtons __dwButtons) 0)
    ((:dwButtonNumber __dwButtonNumber) 0)
    ((:dwPOV __dwPOV) 0)
    )
   (send-super :init)
   (setq _dwXpos (round __dwXpos))
   (setq _dwYpos (round __dwYpos))
   (setq _dwZpos (round __dwZpos))
   (setq _dwRpos (round __dwRpos))
   (setq _dwUpos (round __dwUpos))
   (setq _dwVpos (round __dwVpos))
   (setq _dwButtons (round __dwButtons))
   (setq _dwButtonNumber (round __dwButtonNumber))
   (setq _dwPOV (round __dwPOV))
   self)
  (:dwXpos
   (&optional __dwXpos)
   (if __dwXpos (setq _dwXpos __dwXpos)) _dwXpos)
  (:dwYpos
   (&optional __dwYpos)
   (if __dwYpos (setq _dwYpos __dwYpos)) _dwYpos)
  (:dwZpos
   (&optional __dwZpos)
   (if __dwZpos (setq _dwZpos __dwZpos)) _dwZpos)
  (:dwRpos
   (&optional __dwRpos)
   (if __dwRpos (setq _dwRpos __dwRpos)) _dwRpos)
  (:dwUpos
   (&optional __dwUpos)
   (if __dwUpos (setq _dwUpos __dwUpos)) _dwUpos)
  (:dwVpos
   (&optional __dwVpos)
   (if __dwVpos (setq _dwVpos __dwVpos)) _dwVpos)
  (:dwButtons
   (&optional __dwButtons)
   (if __dwButtons (setq _dwButtons __dwButtons)) _dwButtons)
  (:dwButtonNumber
   (&optional __dwButtonNumber)
   (if __dwButtonNumber (setq _dwButtonNumber __dwButtonNumber)) _dwButtonNumber)
  (:dwPOV
   (&optional __dwPOV)
   (if __dwPOV (setq _dwPOV __dwPOV)) _dwPOV)
  (:serialization-length
   ()
   (+
    ;; uint32 _dwXpos
    4
    ;; uint32 _dwYpos
    4
    ;; uint32 _dwZpos
    4
    ;; uint32 _dwRpos
    4
    ;; uint32 _dwUpos
    4
    ;; uint32 _dwVpos
    4
    ;; uint64 _dwButtons
    8
    ;; uint32 _dwButtonNumber
    4
    ;; uint64 _dwPOV
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _dwXpos
       (write-long _dwXpos s)
     ;; uint32 _dwYpos
       (write-long _dwYpos s)
     ;; uint32 _dwZpos
       (write-long _dwZpos s)
     ;; uint32 _dwRpos
       (write-long _dwRpos s)
     ;; uint32 _dwUpos
       (write-long _dwUpos s)
     ;; uint32 _dwVpos
       (write-long _dwVpos s)
     ;; uint64 _dwButtons
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _dwButtons (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _dwButtons) (= (length (_dwButtons . bv)) 2)) ;; bignum
              (write-long (ash (elt (_dwButtons . bv) 0) 0) s)
              (write-long (ash (elt (_dwButtons . bv) 1) -1) s))
             ((and (class _dwButtons) (= (length (_dwButtons . bv)) 1)) ;; big1
              (write-long (elt (_dwButtons . bv) 0) s)
              (write-long (if (>= _dwButtons 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _dwButtons s)(write-long (if (>= _dwButtons 0) 0 #xffffffff) s)))
     ;; uint32 _dwButtonNumber
       (write-long _dwButtonNumber s)
     ;; uint64 _dwPOV
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _dwPOV (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _dwPOV) (= (length (_dwPOV . bv)) 2)) ;; bignum
              (write-long (ash (elt (_dwPOV . bv) 0) 0) s)
              (write-long (ash (elt (_dwPOV . bv) 1) -1) s))
             ((and (class _dwPOV) (= (length (_dwPOV . bv)) 1)) ;; big1
              (write-long (elt (_dwPOV . bv) 0) s)
              (write-long (if (>= _dwPOV 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _dwPOV s)(write-long (if (>= _dwPOV 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _dwXpos
     (setq _dwXpos (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _dwYpos
     (setq _dwYpos (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _dwZpos
     (setq _dwZpos (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _dwRpos
     (setq _dwRpos (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _dwUpos
     (setq _dwUpos (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _dwVpos
     (setq _dwVpos (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint64 _dwButtons
     
#+(or :alpha :irix6 :x86_64)
      (setf _dwButtons (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _dwButtons (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint32 _dwButtonNumber
     (setq _dwButtonNumber (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint64 _dwPOV
     
#+(or :alpha :irix6 :x86_64)
      (setf _dwPOV (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _dwPOV (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get virtual_device_debugger::joyinfoex :md5sum-) "483386359ee4453093908bee904d04de")
(setf (get virtual_device_debugger::joyinfoex :datatype-) "virtual_device_debugger/joyinfoex")
(setf (get virtual_device_debugger::joyinfoex :definition-)
      "uint32 dwXpos
uint32 dwYpos
uint32 dwZpos
uint32 dwRpos
uint32 dwUpos
uint32 dwVpos
uint64 dwButtons
uint32 dwButtonNumber
uint64 dwPOV


")



(provide :virtual_device_debugger/joyinfoex "483386359ee4453093908bee904d04de")


