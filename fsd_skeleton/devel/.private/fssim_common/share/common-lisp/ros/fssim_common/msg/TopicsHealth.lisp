; Auto-generated. Do not edit!


(cl:in-package fssim_common-msg)


;//! \htmlinclude TopicsHealth.msg.html

(cl:defclass <TopicsHealth> (roslisp-msg-protocol:ros-message)
  ((topics_check_passed
    :reader topics_check_passed
    :initarg :topics_check_passed
    :type cl:boolean
    :initform cl:nil)
   (precision
    :reader precision
    :initarg :precision
    :type cl:float
    :initform 0.0)
   (topics_check
    :reader topics_check
    :initarg :topics_check
    :type (cl:vector fssim_common-msg:TopicState)
   :initform (cl:make-array 0 :element-type 'fssim_common-msg:TopicState :initial-element (cl:make-instance 'fssim_common-msg:TopicState))))
)

(cl:defclass TopicsHealth (<TopicsHealth>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TopicsHealth>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TopicsHealth)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fssim_common-msg:<TopicsHealth> is deprecated: use fssim_common-msg:TopicsHealth instead.")))

(cl:ensure-generic-function 'topics_check_passed-val :lambda-list '(m))
(cl:defmethod topics_check_passed-val ((m <TopicsHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:topics_check_passed-val is deprecated.  Use fssim_common-msg:topics_check_passed instead.")
  (topics_check_passed m))

(cl:ensure-generic-function 'precision-val :lambda-list '(m))
(cl:defmethod precision-val ((m <TopicsHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:precision-val is deprecated.  Use fssim_common-msg:precision instead.")
  (precision m))

(cl:ensure-generic-function 'topics_check-val :lambda-list '(m))
(cl:defmethod topics_check-val ((m <TopicsHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:topics_check-val is deprecated.  Use fssim_common-msg:topics_check instead.")
  (topics_check m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TopicsHealth>) ostream)
  "Serializes a message object of type '<TopicsHealth>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'topics_check_passed) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'precision))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'topics_check))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'topics_check))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TopicsHealth>) istream)
  "Deserializes a message object of type '<TopicsHealth>"
    (cl:setf (cl:slot-value msg 'topics_check_passed) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'precision) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'topics_check) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'topics_check)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'fssim_common-msg:TopicState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TopicsHealth>)))
  "Returns string type for a message object of type '<TopicsHealth>"
  "fssim_common/TopicsHealth")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TopicsHealth)))
  "Returns string type for a message object of type 'TopicsHealth"
  "fssim_common/TopicsHealth")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TopicsHealth>)))
  "Returns md5sum for a message object of type '<TopicsHealth>"
  "2e4a29cd88c13c0624f8c9a144bda96c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TopicsHealth)))
  "Returns md5sum for a message object of type 'TopicsHealth"
  "2e4a29cd88c13c0624f8c9a144bda96c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TopicsHealth>)))
  "Returns full string definition for message of type '<TopicsHealth>"
  (cl:format cl:nil "bool topics_check_passed	# True is all topics passed check~%float32 precision			# How much we allow to deviate topics freq from expected~%TopicState[] topics_check  	# All topics health~%================================================================================~%MSG: fssim_common/TopicState~%string topic_name~%float32 expected_frequency~%float32 measured_frequency~%bool passed~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TopicsHealth)))
  "Returns full string definition for message of type 'TopicsHealth"
  (cl:format cl:nil "bool topics_check_passed	# True is all topics passed check~%float32 precision			# How much we allow to deviate topics freq from expected~%TopicState[] topics_check  	# All topics health~%================================================================================~%MSG: fssim_common/TopicState~%string topic_name~%float32 expected_frequency~%float32 measured_frequency~%bool passed~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TopicsHealth>))
  (cl:+ 0
     1
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'topics_check) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TopicsHealth>))
  "Converts a ROS message object to a list"
  (cl:list 'TopicsHealth
    (cl:cons ':topics_check_passed (topics_check_passed msg))
    (cl:cons ':precision (precision msg))
    (cl:cons ':topics_check (topics_check msg))
))
