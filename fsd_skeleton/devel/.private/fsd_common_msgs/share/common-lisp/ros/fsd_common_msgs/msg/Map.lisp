; Auto-generated. Do not edit!


(cl:in-package fsd_common_msgs-msg)


;//! \htmlinclude Map.msg.html

(cl:defclass <Map> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (cone_yellow
    :reader cone_yellow
    :initarg :cone_yellow
    :type (cl:vector fsd_common_msgs-msg:Cone)
   :initform (cl:make-array 0 :element-type 'fsd_common_msgs-msg:Cone :initial-element (cl:make-instance 'fsd_common_msgs-msg:Cone)))
   (cone_blue
    :reader cone_blue
    :initarg :cone_blue
    :type (cl:vector fsd_common_msgs-msg:Cone)
   :initform (cl:make-array 0 :element-type 'fsd_common_msgs-msg:Cone :initial-element (cl:make-instance 'fsd_common_msgs-msg:Cone)))
   (cone_orange
    :reader cone_orange
    :initarg :cone_orange
    :type (cl:vector fsd_common_msgs-msg:Cone)
   :initform (cl:make-array 0 :element-type 'fsd_common_msgs-msg:Cone :initial-element (cl:make-instance 'fsd_common_msgs-msg:Cone))))
)

(cl:defclass Map (<Map>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Map>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Map)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fsd_common_msgs-msg:<Map> is deprecated: use fsd_common_msgs-msg:Map instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Map>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fsd_common_msgs-msg:header-val is deprecated.  Use fsd_common_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'cone_yellow-val :lambda-list '(m))
(cl:defmethod cone_yellow-val ((m <Map>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fsd_common_msgs-msg:cone_yellow-val is deprecated.  Use fsd_common_msgs-msg:cone_yellow instead.")
  (cone_yellow m))

(cl:ensure-generic-function 'cone_blue-val :lambda-list '(m))
(cl:defmethod cone_blue-val ((m <Map>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fsd_common_msgs-msg:cone_blue-val is deprecated.  Use fsd_common_msgs-msg:cone_blue instead.")
  (cone_blue m))

(cl:ensure-generic-function 'cone_orange-val :lambda-list '(m))
(cl:defmethod cone_orange-val ((m <Map>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fsd_common_msgs-msg:cone_orange-val is deprecated.  Use fsd_common_msgs-msg:cone_orange instead.")
  (cone_orange m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Map>) ostream)
  "Serializes a message object of type '<Map>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cone_yellow))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'cone_yellow))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cone_blue))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'cone_blue))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cone_orange))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'cone_orange))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Map>) istream)
  "Deserializes a message object of type '<Map>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cone_yellow) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cone_yellow)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'fsd_common_msgs-msg:Cone))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cone_blue) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cone_blue)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'fsd_common_msgs-msg:Cone))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cone_orange) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cone_orange)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'fsd_common_msgs-msg:Cone))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Map>)))
  "Returns string type for a message object of type '<Map>"
  "fsd_common_msgs/Map")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Map)))
  "Returns string type for a message object of type 'Map"
  "fsd_common_msgs/Map")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Map>)))
  "Returns md5sum for a message object of type '<Map>"
  "8ddb8bd148fb7730a132faec3d415fe0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Map)))
  "Returns md5sum for a message object of type 'Map"
  "8ddb8bd148fb7730a132faec3d415fe0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Map>)))
  "Returns full string definition for message of type '<Map>"
  (cl:format cl:nil "std_msgs/Header header~%~%fsd_common_msgs/Cone[] cone_yellow~%fsd_common_msgs/Cone[] cone_blue~%fsd_common_msgs/Cone[] cone_orange~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: fsd_common_msgs/Cone~%geometry_msgs/Point position  # coordinate of cone in [x, y]~%std_msgs/String color           # color of cone, 'b' = blue, 'y' = yellow, 'o' = orange~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Map)))
  "Returns full string definition for message of type 'Map"
  (cl:format cl:nil "std_msgs/Header header~%~%fsd_common_msgs/Cone[] cone_yellow~%fsd_common_msgs/Cone[] cone_blue~%fsd_common_msgs/Cone[] cone_orange~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: fsd_common_msgs/Cone~%geometry_msgs/Point position  # coordinate of cone in [x, y]~%std_msgs/String color           # color of cone, 'b' = blue, 'y' = yellow, 'o' = orange~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Map>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cone_yellow) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cone_blue) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cone_orange) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Map>))
  "Converts a ROS message object to a list"
  (cl:list 'Map
    (cl:cons ':header (header msg))
    (cl:cons ':cone_yellow (cone_yellow msg))
    (cl:cons ':cone_blue (cone_blue msg))
    (cl:cons ':cone_orange (cone_orange msg))
))
