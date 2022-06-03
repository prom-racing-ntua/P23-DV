; Auto-generated. Do not edit!


(cl:in-package fssim_common-msg)


;//! \htmlinclude TopicState.msg.html

(cl:defclass <TopicState> (roslisp-msg-protocol:ros-message)
  ((topic_name
    :reader topic_name
    :initarg :topic_name
    :type cl:string
    :initform "")
   (expected_frequency
    :reader expected_frequency
    :initarg :expected_frequency
    :type cl:float
    :initform 0.0)
   (measured_frequency
    :reader measured_frequency
    :initarg :measured_frequency
    :type cl:float
    :initform 0.0)
   (passed
    :reader passed
    :initarg :passed
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass TopicState (<TopicState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TopicState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TopicState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fssim_common-msg:<TopicState> is deprecated: use fssim_common-msg:TopicState instead.")))

(cl:ensure-generic-function 'topic_name-val :lambda-list '(m))
(cl:defmethod topic_name-val ((m <TopicState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:topic_name-val is deprecated.  Use fssim_common-msg:topic_name instead.")
  (topic_name m))

(cl:ensure-generic-function 'expected_frequency-val :lambda-list '(m))
(cl:defmethod expected_frequency-val ((m <TopicState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:expected_frequency-val is deprecated.  Use fssim_common-msg:expected_frequency instead.")
  (expected_frequency m))

(cl:ensure-generic-function 'measured_frequency-val :lambda-list '(m))
(cl:defmethod measured_frequency-val ((m <TopicState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:measured_frequency-val is deprecated.  Use fssim_common-msg:measured_frequency instead.")
  (measured_frequency m))

(cl:ensure-generic-function 'passed-val :lambda-list '(m))
(cl:defmethod passed-val ((m <TopicState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:passed-val is deprecated.  Use fssim_common-msg:passed instead.")
  (passed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TopicState>) ostream)
  "Serializes a message object of type '<TopicState>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'topic_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'topic_name))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'expected_frequency))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'measured_frequency))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'passed) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TopicState>) istream)
  "Deserializes a message object of type '<TopicState>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'topic_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'topic_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'expected_frequency) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'measured_frequency) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'passed) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TopicState>)))
  "Returns string type for a message object of type '<TopicState>"
  "fssim_common/TopicState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TopicState)))
  "Returns string type for a message object of type 'TopicState"
  "fssim_common/TopicState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TopicState>)))
  "Returns md5sum for a message object of type '<TopicState>"
  "5557167df4d3920fba79516729b9f245")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TopicState)))
  "Returns md5sum for a message object of type 'TopicState"
  "5557167df4d3920fba79516729b9f245")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TopicState>)))
  "Returns full string definition for message of type '<TopicState>"
  (cl:format cl:nil "string topic_name~%float32 expected_frequency~%float32 measured_frequency~%bool passed~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TopicState)))
  "Returns full string definition for message of type 'TopicState"
  (cl:format cl:nil "string topic_name~%float32 expected_frequency~%float32 measured_frequency~%bool passed~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TopicState>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'topic_name))
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TopicState>))
  "Converts a ROS message object to a list"
  (cl:list 'TopicState
    (cl:cons ':topic_name (topic_name msg))
    (cl:cons ':expected_frequency (expected_frequency msg))
    (cl:cons ':measured_frequency (measured_frequency msg))
    (cl:cons ':passed (passed msg))
))
