; Auto-generated. Do not edit!


(cl:in-package fssim_common-msg)


;//! \htmlinclude Track.msg.html

(cl:defclass <Track> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (cones_left
    :reader cones_left
    :initarg :cones_left
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (cones_right
    :reader cones_right
    :initarg :cones_right
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (cones_orange
    :reader cones_orange
    :initarg :cones_orange
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (cones_orange_big
    :reader cones_orange_big
    :initarg :cones_orange_big
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (tk_device_start
    :reader tk_device_start
    :initarg :tk_device_start
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (tk_device_end
    :reader tk_device_end
    :initarg :tk_device_end
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass Track (<Track>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Track>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Track)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fssim_common-msg:<Track> is deprecated: use fssim_common-msg:Track instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Track>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:header-val is deprecated.  Use fssim_common-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'cones_left-val :lambda-list '(m))
(cl:defmethod cones_left-val ((m <Track>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:cones_left-val is deprecated.  Use fssim_common-msg:cones_left instead.")
  (cones_left m))

(cl:ensure-generic-function 'cones_right-val :lambda-list '(m))
(cl:defmethod cones_right-val ((m <Track>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:cones_right-val is deprecated.  Use fssim_common-msg:cones_right instead.")
  (cones_right m))

(cl:ensure-generic-function 'cones_orange-val :lambda-list '(m))
(cl:defmethod cones_orange-val ((m <Track>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:cones_orange-val is deprecated.  Use fssim_common-msg:cones_orange instead.")
  (cones_orange m))

(cl:ensure-generic-function 'cones_orange_big-val :lambda-list '(m))
(cl:defmethod cones_orange_big-val ((m <Track>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:cones_orange_big-val is deprecated.  Use fssim_common-msg:cones_orange_big instead.")
  (cones_orange_big m))

(cl:ensure-generic-function 'tk_device_start-val :lambda-list '(m))
(cl:defmethod tk_device_start-val ((m <Track>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:tk_device_start-val is deprecated.  Use fssim_common-msg:tk_device_start instead.")
  (tk_device_start m))

(cl:ensure-generic-function 'tk_device_end-val :lambda-list '(m))
(cl:defmethod tk_device_end-val ((m <Track>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:tk_device_end-val is deprecated.  Use fssim_common-msg:tk_device_end instead.")
  (tk_device_end m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Track>) ostream)
  "Serializes a message object of type '<Track>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cones_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'cones_left))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cones_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'cones_right))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cones_orange))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'cones_orange))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cones_orange_big))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'cones_orange_big))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tk_device_start))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tk_device_start))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tk_device_end))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'tk_device_end))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Track>) istream)
  "Deserializes a message object of type '<Track>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cones_left) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cones_left)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cones_right) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cones_right)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cones_orange) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cones_orange)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cones_orange_big) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cones_orange_big)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tk_device_start) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tk_device_start)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tk_device_end) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tk_device_end)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Track>)))
  "Returns string type for a message object of type '<Track>"
  "fssim_common/Track")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Track)))
  "Returns string type for a message object of type 'Track"
  "fssim_common/Track")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Track>)))
  "Returns md5sum for a message object of type '<Track>"
  "b3e47dce05b0e0c04dc61079e1408cdf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Track)))
  "Returns md5sum for a message object of type 'Track"
  "b3e47dce05b0e0c04dc61079e1408cdf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Track>)))
  "Returns full string definition for message of type '<Track>"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Point[] cones_left~%geometry_msgs/Point[] cones_right~%geometry_msgs/Point[] cones_orange~%geometry_msgs/Point[] cones_orange_big~%geometry_msgs/Point[] tk_device_start~%geometry_msgs/Point[] tk_device_end~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Track)))
  "Returns full string definition for message of type 'Track"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Point[] cones_left~%geometry_msgs/Point[] cones_right~%geometry_msgs/Point[] cones_orange~%geometry_msgs/Point[] cones_orange_big~%geometry_msgs/Point[] tk_device_start~%geometry_msgs/Point[] tk_device_end~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Track>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cones_left) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cones_right) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cones_orange) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cones_orange_big) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tk_device_start) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tk_device_end) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Track>))
  "Converts a ROS message object to a list"
  (cl:list 'Track
    (cl:cons ':header (header msg))
    (cl:cons ':cones_left (cones_left msg))
    (cl:cons ':cones_right (cones_right msg))
    (cl:cons ':cones_orange (cones_orange msg))
    (cl:cons ':cones_orange_big (cones_orange_big msg))
    (cl:cons ':tk_device_start (tk_device_start msg))
    (cl:cons ':tk_device_end (tk_device_end msg))
))
