;; Auto-generated. Do not edit!


(when (boundp 'fssim_common::TopicState)
  (if (not (find-package "FSSIM_COMMON"))
    (make-package "FSSIM_COMMON"))
  (shadow 'TopicState (find-package "FSSIM_COMMON")))
(unless (find-package "FSSIM_COMMON::TOPICSTATE")
  (make-package "FSSIM_COMMON::TOPICSTATE"))

(in-package "ROS")
;;//! \htmlinclude TopicState.msg.html


(defclass fssim_common::TopicState
  :super ros::object
  :slots (_topic_name _expected_frequency _measured_frequency _passed ))

(defmethod fssim_common::TopicState
  (:init
   (&key
    ((:topic_name __topic_name) "")
    ((:expected_frequency __expected_frequency) 0.0)
    ((:measured_frequency __measured_frequency) 0.0)
    ((:passed __passed) nil)
    )
   (send-super :init)
   (setq _topic_name (string __topic_name))
   (setq _expected_frequency (float __expected_frequency))
   (setq _measured_frequency (float __measured_frequency))
   (setq _passed __passed)
   self)
  (:topic_name
   (&optional __topic_name)
   (if __topic_name (setq _topic_name __topic_name)) _topic_name)
  (:expected_frequency
   (&optional __expected_frequency)
   (if __expected_frequency (setq _expected_frequency __expected_frequency)) _expected_frequency)
  (:measured_frequency
   (&optional __measured_frequency)
   (if __measured_frequency (setq _measured_frequency __measured_frequency)) _measured_frequency)
  (:passed
   (&optional (__passed :null))
   (if (not (eq __passed :null)) (setq _passed __passed)) _passed)
  (:serialization-length
   ()
   (+
    ;; string _topic_name
    4 (length _topic_name)
    ;; float32 _expected_frequency
    4
    ;; float32 _measured_frequency
    4
    ;; bool _passed
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _topic_name
       (write-long (length _topic_name) s) (princ _topic_name s)
     ;; float32 _expected_frequency
       (sys::poke _expected_frequency (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _measured_frequency
       (sys::poke _measured_frequency (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _passed
       (if _passed (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _topic_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _topic_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _expected_frequency
     (setq _expected_frequency (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _measured_frequency
     (setq _measured_frequency (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _passed
     (setq _passed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get fssim_common::TopicState :md5sum-) "5557167df4d3920fba79516729b9f245")
(setf (get fssim_common::TopicState :datatype-) "fssim_common/TopicState")
(setf (get fssim_common::TopicState :definition-)
      "string topic_name
float32 expected_frequency
float32 measured_frequency
bool passed
")



(provide :fssim_common/TopicState "5557167df4d3920fba79516729b9f245")


