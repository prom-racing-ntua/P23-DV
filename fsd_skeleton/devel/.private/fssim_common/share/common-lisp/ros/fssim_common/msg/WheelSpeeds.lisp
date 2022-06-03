; Auto-generated. Do not edit!


(cl:in-package fssim_common-msg)


;//! \htmlinclude WheelSpeeds.msg.html

(cl:defclass <WheelSpeeds> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (version
    :reader version
    :initarg :version
    :type cl:fixnum
    :initform 0)
   (rpm_front_left
    :reader rpm_front_left
    :initarg :rpm_front_left
    :type cl:fixnum
    :initform 0)
   (rpm_front_right
    :reader rpm_front_right
    :initarg :rpm_front_right
    :type cl:fixnum
    :initform 0)
   (rpm_rear_left
    :reader rpm_rear_left
    :initarg :rpm_rear_left
    :type cl:fixnum
    :initform 0)
   (rpm_rear_right
    :reader rpm_rear_right
    :initarg :rpm_rear_right
    :type cl:fixnum
    :initform 0))
)

(cl:defclass WheelSpeeds (<WheelSpeeds>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WheelSpeeds>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WheelSpeeds)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fssim_common-msg:<WheelSpeeds> is deprecated: use fssim_common-msg:WheelSpeeds instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <WheelSpeeds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:header-val is deprecated.  Use fssim_common-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'version-val :lambda-list '(m))
(cl:defmethod version-val ((m <WheelSpeeds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:version-val is deprecated.  Use fssim_common-msg:version instead.")
  (version m))

(cl:ensure-generic-function 'rpm_front_left-val :lambda-list '(m))
(cl:defmethod rpm_front_left-val ((m <WheelSpeeds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:rpm_front_left-val is deprecated.  Use fssim_common-msg:rpm_front_left instead.")
  (rpm_front_left m))

(cl:ensure-generic-function 'rpm_front_right-val :lambda-list '(m))
(cl:defmethod rpm_front_right-val ((m <WheelSpeeds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:rpm_front_right-val is deprecated.  Use fssim_common-msg:rpm_front_right instead.")
  (rpm_front_right m))

(cl:ensure-generic-function 'rpm_rear_left-val :lambda-list '(m))
(cl:defmethod rpm_rear_left-val ((m <WheelSpeeds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:rpm_rear_left-val is deprecated.  Use fssim_common-msg:rpm_rear_left instead.")
  (rpm_rear_left m))

(cl:ensure-generic-function 'rpm_rear_right-val :lambda-list '(m))
(cl:defmethod rpm_rear_right-val ((m <WheelSpeeds>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:rpm_rear_right-val is deprecated.  Use fssim_common-msg:rpm_rear_right instead.")
  (rpm_rear_right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WheelSpeeds>) ostream)
  "Serializes a message object of type '<WheelSpeeds>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'version)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'rpm_front_left)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'rpm_front_right)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'rpm_rear_left)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'rpm_rear_right)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WheelSpeeds>) istream)
  "Deserializes a message object of type '<WheelSpeeds>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'version)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rpm_front_left) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rpm_front_right) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rpm_rear_left) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rpm_rear_right) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WheelSpeeds>)))
  "Returns string type for a message object of type '<WheelSpeeds>"
  "fssim_common/WheelSpeeds")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WheelSpeeds)))
  "Returns string type for a message object of type 'WheelSpeeds"
  "fssim_common/WheelSpeeds")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WheelSpeeds>)))
  "Returns md5sum for a message object of type '<WheelSpeeds>"
  "de6e76c895b1095f899172fc46f64a60")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WheelSpeeds)))
  "Returns md5sum for a message object of type 'WheelSpeeds"
  "de6e76c895b1095f899172fc46f64a60")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WheelSpeeds>)))
  "Returns full string definition for message of type '<WheelSpeeds>"
  (cl:format cl:nil "# Header~%std_msgs/Header header~%uint8 version~%~%int16 rpm_front_left~%int16 rpm_front_right~%int16 rpm_rear_left~%int16 rpm_rear_right~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WheelSpeeds)))
  "Returns full string definition for message of type 'WheelSpeeds"
  (cl:format cl:nil "# Header~%std_msgs/Header header~%uint8 version~%~%int16 rpm_front_left~%int16 rpm_front_right~%int16 rpm_rear_left~%int16 rpm_rear_right~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WheelSpeeds>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WheelSpeeds>))
  "Converts a ROS message object to a list"
  (cl:list 'WheelSpeeds
    (cl:cons ':header (header msg))
    (cl:cons ':version (version msg))
    (cl:cons ':rpm_front_left (rpm_front_left msg))
    (cl:cons ':rpm_front_right (rpm_front_right msg))
    (cl:cons ':rpm_rear_left (rpm_rear_left msg))
    (cl:cons ':rpm_rear_right (rpm_rear_right msg))
))
