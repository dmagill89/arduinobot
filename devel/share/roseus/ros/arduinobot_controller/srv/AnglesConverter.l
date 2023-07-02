;; Auto-generated. Do not edit!


(when (boundp 'arduinobot_controller::AnglesConverter)
  (if (not (find-package "ARDUINOBOT_CONTROLLER"))
    (make-package "ARDUINOBOT_CONTROLLER"))
  (shadow 'AnglesConverter (find-package "ARDUINOBOT_CONTROLLER")))
(unless (find-package "ARDUINOBOT_CONTROLLER::ANGLESCONVERTER")
  (make-package "ARDUINOBOT_CONTROLLER::ANGLESCONVERTER"))
(unless (find-package "ARDUINOBOT_CONTROLLER::ANGLESCONVERTERREQUEST")
  (make-package "ARDUINOBOT_CONTROLLER::ANGLESCONVERTERREQUEST"))
(unless (find-package "ARDUINOBOT_CONTROLLER::ANGLESCONVERTERRESPONSE")
  (make-package "ARDUINOBOT_CONTROLLER::ANGLESCONVERTERRESPONSE"))

(in-package "ROS")





(defclass arduinobot_controller::AnglesConverterRequest
  :super ros::object
  :slots (_base _shoulder _elbow _gripper ))

(defmethod arduinobot_controller::AnglesConverterRequest
  (:init
   (&key
    ((:base __base) 0.0)
    ((:shoulder __shoulder) 0.0)
    ((:elbow __elbow) 0.0)
    ((:gripper __gripper) 0.0)
    )
   (send-super :init)
   (setq _base (float __base))
   (setq _shoulder (float __shoulder))
   (setq _elbow (float __elbow))
   (setq _gripper (float __gripper))
   self)
  (:base
   (&optional __base)
   (if __base (setq _base __base)) _base)
  (:shoulder
   (&optional __shoulder)
   (if __shoulder (setq _shoulder __shoulder)) _shoulder)
  (:elbow
   (&optional __elbow)
   (if __elbow (setq _elbow __elbow)) _elbow)
  (:gripper
   (&optional __gripper)
   (if __gripper (setq _gripper __gripper)) _gripper)
  (:serialization-length
   ()
   (+
    ;; float64 _base
    8
    ;; float64 _shoulder
    8
    ;; float64 _elbow
    8
    ;; float64 _gripper
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _base
       (sys::poke _base (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _shoulder
       (sys::poke _shoulder (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _elbow
       (sys::poke _elbow (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _gripper
       (sys::poke _gripper (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _base
     (setq _base (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _shoulder
     (setq _shoulder (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _elbow
     (setq _elbow (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _gripper
     (setq _gripper (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass arduinobot_controller::AnglesConverterResponse
  :super ros::object
  :slots (_base _shoulder _elbow _gripper ))

(defmethod arduinobot_controller::AnglesConverterResponse
  (:init
   (&key
    ((:base __base) 0.0)
    ((:shoulder __shoulder) 0.0)
    ((:elbow __elbow) 0.0)
    ((:gripper __gripper) 0.0)
    )
   (send-super :init)
   (setq _base (float __base))
   (setq _shoulder (float __shoulder))
   (setq _elbow (float __elbow))
   (setq _gripper (float __gripper))
   self)
  (:base
   (&optional __base)
   (if __base (setq _base __base)) _base)
  (:shoulder
   (&optional __shoulder)
   (if __shoulder (setq _shoulder __shoulder)) _shoulder)
  (:elbow
   (&optional __elbow)
   (if __elbow (setq _elbow __elbow)) _elbow)
  (:gripper
   (&optional __gripper)
   (if __gripper (setq _gripper __gripper)) _gripper)
  (:serialization-length
   ()
   (+
    ;; float64 _base
    8
    ;; float64 _shoulder
    8
    ;; float64 _elbow
    8
    ;; float64 _gripper
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _base
       (sys::poke _base (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _shoulder
       (sys::poke _shoulder (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _elbow
       (sys::poke _elbow (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _gripper
       (sys::poke _gripper (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _base
     (setq _base (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _shoulder
     (setq _shoulder (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _elbow
     (setq _elbow (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _gripper
     (setq _gripper (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass arduinobot_controller::AnglesConverter
  :super ros::object
  :slots ())

(setf (get arduinobot_controller::AnglesConverter :md5sum-) "904e8346ace63ec441f6f58d8efc6687")
(setf (get arduinobot_controller::AnglesConverter :datatype-) "arduinobot_controller/AnglesConverter")
(setf (get arduinobot_controller::AnglesConverter :request) arduinobot_controller::AnglesConverterRequest)
(setf (get arduinobot_controller::AnglesConverter :response) arduinobot_controller::AnglesConverterResponse)

(defmethod arduinobot_controller::AnglesConverterRequest
  (:response () (instance arduinobot_controller::AnglesConverterResponse :init)))

(setf (get arduinobot_controller::AnglesConverterRequest :md5sum-) "904e8346ace63ec441f6f58d8efc6687")
(setf (get arduinobot_controller::AnglesConverterRequest :datatype-) "arduinobot_controller/AnglesConverterRequest")
(setf (get arduinobot_controller::AnglesConverterRequest :definition-)
      "# Request 
float64 base
float64 shoulder
float64 elbow
float64 gripper
---
# Response
float64 base
float64 shoulder
float64 elbow
float64 gripper
")

(setf (get arduinobot_controller::AnglesConverterResponse :md5sum-) "904e8346ace63ec441f6f58d8efc6687")
(setf (get arduinobot_controller::AnglesConverterResponse :datatype-) "arduinobot_controller/AnglesConverterResponse")
(setf (get arduinobot_controller::AnglesConverterResponse :definition-)
      "# Request 
float64 base
float64 shoulder
float64 elbow
float64 gripper
---
# Response
float64 base
float64 shoulder
float64 elbow
float64 gripper
")



(provide :arduinobot_controller/AnglesConverter "904e8346ace63ec441f6f58d8efc6687")


