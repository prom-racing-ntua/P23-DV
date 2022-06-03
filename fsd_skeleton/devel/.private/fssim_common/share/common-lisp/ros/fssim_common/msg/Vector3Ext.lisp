; Auto-generated. Do not edit!


(cl:in-package fssim_common-msg)


;//! \htmlinclude Vector3Ext.msg.html

(cl:defclass <Vector3Ext> (roslisp-msg-protocol:ros-message)
  ((vec
    :reader vec
    :initarg :vec
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (mag
    :reader mag
    :initarg :mag
    :type cl:float
    :initform 0.0))
)

(cl:defclass Vector3Ext (<Vector3Ext>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Vector3Ext>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Vector3Ext)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fssim_common-msg:<Vector3Ext> is deprecated: use fssim_common-msg:Vector3Ext instead.")))

(cl:ensure-generic-function 'vec-val :lambda-list '(m))
(cl:defmethod vec-val ((m <Vector3Ext>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:vec-val is deprecated.  Use fssim_common-msg:vec instead.")
  (vec m))

(cl:ensure-generic-function 'mag-val :lambda-list '(m))
(cl:defmethod mag-val ((m <Vector3Ext>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:mag-val is deprecated.  Use fssim_common-msg:mag instead.")
  (mag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Vector3Ext>) ostream)
  "Serializes a message object of type '<Vector3Ext>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vec) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'mag))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Vector3Ext>) istream)
  "Deserializes a message object of type '<Vector3Ext>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vec) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mag) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Vector3Ext>)))
  "Returns string type for a message object of type '<Vector3Ext>"
  "fssim_common/Vector3Ext")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Vector3Ext)))
  "Returns string type for a message object of type 'Vector3Ext"
  "fssim_common/Vector3Ext")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Vector3Ext>)))
  "Returns md5sum for a message object of type '<Vector3Ext>"
  "9ce7c2ac065b8145af6ba1b53af0b0bd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Vector3Ext)))
  "Returns md5sum for a message object of type 'Vector3Ext"
  "9ce7c2ac065b8145af6ba1b53af0b0bd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Vector3Ext>)))
  "Returns full string definition for message of type '<Vector3Ext>"
  (cl:format cl:nil "geometry_msgs/Vector3 vec~%float64 mag~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Vector3Ext)))
  "Returns full string definition for message of type 'Vector3Ext"
  (cl:format cl:nil "geometry_msgs/Vector3 vec~%float64 mag~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Vector3Ext>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vec))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Vector3Ext>))
  "Converts a ROS message object to a list"
  (cl:list 'Vector3Ext
    (cl:cons ':vec (vec msg))
    (cl:cons ':mag (mag msg))
))
