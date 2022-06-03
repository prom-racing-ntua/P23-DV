;; Auto-generated. Do not edit!


(when (boundp 'fssim_common::SimHealth)
  (if (not (find-package "FSSIM_COMMON"))
    (make-package "FSSIM_COMMON"))
  (shadow 'SimHealth (find-package "FSSIM_COMMON")))
(unless (find-package "FSSIM_COMMON::SIMHEALTH")
  (make-package "FSSIM_COMMON::SIMHEALTH"))

(in-package "ROS")
;;//! \htmlinclude SimHealth.msg.html


(defclass fssim_common::SimHealth
  :super ros::object
  :slots (_request_shutdown _vehicle_started _topics_health ))

(defmethod fssim_common::SimHealth
  (:init
   (&key
    ((:request_shutdown __request_shutdown) nil)
    ((:vehicle_started __vehicle_started) nil)
    ((:topics_health __topics_health) (instance fssim_common::TopicsHealth :init))
    )
   (send-super :init)
   (setq _request_shutdown __request_shutdown)
   (setq _vehicle_started __vehicle_started)
   (setq _topics_health __topics_health)
   self)
  (:request_shutdown
   (&optional (__request_shutdown :null))
   (if (not (eq __request_shutdown :null)) (setq _request_shutdown __request_shutdown)) _request_shutdown)
  (:vehicle_started
   (&optional (__vehicle_started :null))
   (if (not (eq __vehicle_started :null)) (setq _vehicle_started __vehicle_started)) _vehicle_started)
  (:topics_health
   (&rest __topics_health)
   (if (keywordp (car __topics_health))
       (send* _topics_health __topics_health)
     (progn
       (if __topics_health (setq _topics_health (car __topics_health)))
       _topics_health)))
  (:serialization-length
   ()
   (+
    ;; bool _request_shutdown
    1
    ;; bool _vehicle_started
    1
    ;; fssim_common/TopicsHealth _topics_health
    (send _topics_health :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _request_shutdown
       (if _request_shutdown (write-byte -1 s) (write-byte 0 s))
     ;; bool _vehicle_started
       (if _vehicle_started (write-byte -1 s) (write-byte 0 s))
     ;; fssim_common/TopicsHealth _topics_health
       (send _topics_health :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _request_shutdown
     (setq _request_shutdown (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _vehicle_started
     (setq _vehicle_started (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; fssim_common/TopicsHealth _topics_health
     (send _topics_health :deserialize buf ptr-) (incf ptr- (send _topics_health :serialization-length))
   ;;
   self)
  )

(setf (get fssim_common::SimHealth :md5sum-) "2d6de42391271dba371094f7524b84b6")
(setf (get fssim_common::SimHealth :datatype-) "fssim_common/SimHealth")
(setf (get fssim_common::SimHealth :definition-)
      "bool request_shutdown 	# If TRUE we request immidiate shutdown
bool vehicle_started	# We send res message to start the vehicle	

TopicsHealth topics_health # List of topics and their health
================================================================================
MSG: fssim_common/TopicsHealth
bool topics_check_passed	# True is all topics passed check
float32 precision			# How much we allow to deviate topics freq from expected
TopicState[] topics_check  	# All topics health
================================================================================
MSG: fssim_common/TopicState
string topic_name
float32 expected_frequency
float32 measured_frequency
bool passed
")



(provide :fssim_common/SimHealth "2d6de42391271dba371094f7524b84b6")


