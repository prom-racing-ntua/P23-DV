; Auto-generated. Do not edit!


(cl:in-package fssim_common-msg)


;//! \htmlinclude SimHealth.msg.html

(cl:defclass <SimHealth> (roslisp-msg-protocol:ros-message)
  ((request_shutdown
    :reader request_shutdown
    :initarg :request_shutdown
    :type cl:boolean
    :initform cl:nil)
   (vehicle_started
    :reader vehicle_started
    :initarg :vehicle_started
    :type cl:boolean
    :initform cl:nil)
   (topics_health
    :reader topics_health
    :initarg :topics_health
    :type fssim_common-msg:TopicsHealth
    :initform (cl:make-instance 'fssim_common-msg:TopicsHealth)))
)

(cl:defclass SimHealth (<SimHealth>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SimHealth>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SimHealth)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fssim_common-msg:<SimHealth> is deprecated: use fssim_common-msg:SimHealth instead.")))

(cl:ensure-generic-function 'request_shutdown-val :lambda-list '(m))
(cl:defmethod request_shutdown-val ((m <SimHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:request_shutdown-val is deprecated.  Use fssim_common-msg:request_shutdown instead.")
  (request_shutdown m))

(cl:ensure-generic-function 'vehicle_started-val :lambda-list '(m))
(cl:defmethod vehicle_started-val ((m <SimHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:vehicle_started-val is deprecated.  Use fssim_common-msg:vehicle_started instead.")
  (vehicle_started m))

(cl:ensure-generic-function 'topics_health-val :lambda-list '(m))
(cl:defmethod topics_health-val ((m <SimHealth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:topics_health-val is deprecated.  Use fssim_common-msg:topics_health instead.")
  (topics_health m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SimHealth>) ostream)
  "Serializes a message object of type '<SimHealth>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'request_shutdown) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'vehicle_started) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'topics_health) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SimHealth>) istream)
  "Deserializes a message object of type '<SimHealth>"
    (cl:setf (cl:slot-value msg 'request_shutdown) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'vehicle_started) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'topics_health) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SimHealth>)))
  "Returns string type for a message object of type '<SimHealth>"
  "fssim_common/SimHealth")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SimHealth)))
  "Returns string type for a message object of type 'SimHealth"
  "fssim_common/SimHealth")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SimHealth>)))
  "Returns md5sum for a message object of type '<SimHealth>"
  "2d6de42391271dba371094f7524b84b6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SimHealth)))
  "Returns md5sum for a message object of type 'SimHealth"
  "2d6de42391271dba371094f7524b84b6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SimHealth>)))
  "Returns full string definition for message of type '<SimHealth>"
  (cl:format cl:nil "bool request_shutdown 	# If TRUE we request immidiate shutdown~%bool vehicle_started	# We send res message to start the vehicle	~%~%TopicsHealth topics_health # List of topics and their health~%================================================================================~%MSG: fssim_common/TopicsHealth~%bool topics_check_passed	# True is all topics passed check~%float32 precision			# How much we allow to deviate topics freq from expected~%TopicState[] topics_check  	# All topics health~%================================================================================~%MSG: fssim_common/TopicState~%string topic_name~%float32 expected_frequency~%float32 measured_frequency~%bool passed~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SimHealth)))
  "Returns full string definition for message of type 'SimHealth"
  (cl:format cl:nil "bool request_shutdown 	# If TRUE we request immidiate shutdown~%bool vehicle_started	# We send res message to start the vehicle	~%~%TopicsHealth topics_health # List of topics and their health~%================================================================================~%MSG: fssim_common/TopicsHealth~%bool topics_check_passed	# True is all topics passed check~%float32 precision			# How much we allow to deviate topics freq from expected~%TopicState[] topics_check  	# All topics health~%================================================================================~%MSG: fssim_common/TopicState~%string topic_name~%float32 expected_frequency~%float32 measured_frequency~%bool passed~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SimHealth>))
  (cl:+ 0
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'topics_health))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SimHealth>))
  "Converts a ROS message object to a list"
  (cl:list 'SimHealth
    (cl:cons ':request_shutdown (request_shutdown msg))
    (cl:cons ':vehicle_started (vehicle_started msg))
    (cl:cons ':topics_health (topics_health msg))
))
