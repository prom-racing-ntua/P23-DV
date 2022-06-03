;; Auto-generated. Do not edit!


(when (boundp 'fssim_common::TopicsHealth)
  (if (not (find-package "FSSIM_COMMON"))
    (make-package "FSSIM_COMMON"))
  (shadow 'TopicsHealth (find-package "FSSIM_COMMON")))
(unless (find-package "FSSIM_COMMON::TOPICSHEALTH")
  (make-package "FSSIM_COMMON::TOPICSHEALTH"))

(in-package "ROS")
;;//! \htmlinclude TopicsHealth.msg.html


(defclass fssim_common::TopicsHealth
  :super ros::object
  :slots (_topics_check_passed _precision _topics_check ))

(defmethod fssim_common::TopicsHealth
  (:init
   (&key
    ((:topics_check_passed __topics_check_passed) nil)
    ((:precision __precision) 0.0)
    ((:topics_check __topics_check) ())
    )
   (send-super :init)
   (setq _topics_check_passed __topics_check_passed)
   (setq _precision (float __precision))
   (setq _topics_check __topics_check)
   self)
  (:topics_check_passed
   (&optional (__topics_check_passed :null))
   (if (not (eq __topics_check_passed :null)) (setq _topics_check_passed __topics_check_passed)) _topics_check_passed)
  (:precision
   (&optional __precision)
   (if __precision (setq _precision __precision)) _precision)
  (:topics_check
   (&rest __topics_check)
   (if (keywordp (car __topics_check))
       (send* _topics_check __topics_check)
     (progn
       (if __topics_check (setq _topics_check (car __topics_check)))
       _topics_check)))
  (:serialization-length
   ()
   (+
    ;; bool _topics_check_passed
    1
    ;; float32 _precision
    4
    ;; fssim_common/TopicState[] _topics_check
    (apply #'+ (send-all _topics_check :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _topics_check_passed
       (if _topics_check_passed (write-byte -1 s) (write-byte 0 s))
     ;; float32 _precision
       (sys::poke _precision (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; fssim_common/TopicState[] _topics_check
     (write-long (length _topics_check) s)
     (dolist (elem _topics_check)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _topics_check_passed
     (setq _topics_check_passed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _precision
     (setq _precision (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; fssim_common/TopicState[] _topics_check
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _topics_check (let (r) (dotimes (i n) (push (instance fssim_common::TopicState :init) r)) r))
     (dolist (elem- _topics_check)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get fssim_common::TopicsHealth :md5sum-) "2e4a29cd88c13c0624f8c9a144bda96c")
(setf (get fssim_common::TopicsHealth :datatype-) "fssim_common/TopicsHealth")
(setf (get fssim_common::TopicsHealth :definition-)
      "bool topics_check_passed	# True is all topics passed check
float32 precision			# How much we allow to deviate topics freq from expected
TopicState[] topics_check  	# All topics health
================================================================================
MSG: fssim_common/TopicState
string topic_name
float32 expected_frequency
float32 measured_frequency
bool passed
")



(provide :fssim_common/TopicsHealth "2e4a29cd88c13c0624f8c9a144bda96c")


