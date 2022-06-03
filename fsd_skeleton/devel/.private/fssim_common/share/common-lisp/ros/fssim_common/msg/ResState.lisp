; Auto-generated. Do not edit!


(cl:in-package fssim_common-msg)


;//! \htmlinclude ResState.msg.html

(cl:defclass <ResState> (roslisp-msg-protocol:ros-message)
  ((emergency
    :reader emergency
    :initarg :emergency
    :type cl:boolean
    :initform cl:nil)
   (on_off_switch
    :reader on_off_switch
    :initarg :on_off_switch
    :type cl:boolean
    :initform cl:nil)
   (push_button
    :reader push_button
    :initarg :push_button
    :type cl:boolean
    :initform cl:nil)
   (communication_interrupted
    :reader communication_interrupted
    :initarg :communication_interrupted
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ResState (<ResState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fssim_common-msg:<ResState> is deprecated: use fssim_common-msg:ResState instead.")))

(cl:ensure-generic-function 'emergency-val :lambda-list '(m))
(cl:defmethod emergency-val ((m <ResState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:emergency-val is deprecated.  Use fssim_common-msg:emergency instead.")
  (emergency m))

(cl:ensure-generic-function 'on_off_switch-val :lambda-list '(m))
(cl:defmethod on_off_switch-val ((m <ResState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:on_off_switch-val is deprecated.  Use fssim_common-msg:on_off_switch instead.")
  (on_off_switch m))

(cl:ensure-generic-function 'push_button-val :lambda-list '(m))
(cl:defmethod push_button-val ((m <ResState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:push_button-val is deprecated.  Use fssim_common-msg:push_button instead.")
  (push_button m))

(cl:ensure-generic-function 'communication_interrupted-val :lambda-list '(m))
(cl:defmethod communication_interrupted-val ((m <ResState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fssim_common-msg:communication_interrupted-val is deprecated.  Use fssim_common-msg:communication_interrupted instead.")
  (communication_interrupted m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResState>) ostream)
  "Serializes a message object of type '<ResState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'emergency) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'on_off_switch) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'push_button) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'communication_interrupted) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResState>) istream)
  "Deserializes a message object of type '<ResState>"
    (cl:setf (cl:slot-value msg 'emergency) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'on_off_switch) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'push_button) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'communication_interrupted) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResState>)))
  "Returns string type for a message object of type '<ResState>"
  "fssim_common/ResState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResState)))
  "Returns string type for a message object of type 'ResState"
  "fssim_common/ResState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResState>)))
  "Returns md5sum for a message object of type '<ResState>"
  "2c68d942044efe0714c25879acd65327")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResState)))
  "Returns md5sum for a message object of type 'ResState"
  "2c68d942044efe0714c25879acd65327")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResState>)))
  "Returns full string definition for message of type '<ResState>"
  (cl:format cl:nil "# State of the three buttons on the RES~%bool emergency 					# 0 = OK, 1 = Emergency~%bool on_off_switch 				# 0 = 0, 1 = 1 (Comments couldn't be more helpful :p )~%bool push_button 				# 1 = pressed~%# RES will trigger emergency 200ms after this becomes 1 (unless communication is recovered)~%bool communication_interrupted 	# 0 = OK, 1 = Interrupted~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResState)))
  "Returns full string definition for message of type 'ResState"
  (cl:format cl:nil "# State of the three buttons on the RES~%bool emergency 					# 0 = OK, 1 = Emergency~%bool on_off_switch 				# 0 = 0, 1 = 1 (Comments couldn't be more helpful :p )~%bool push_button 				# 1 = pressed~%# RES will trigger emergency 200ms after this becomes 1 (unless communication is recovered)~%bool communication_interrupted 	# 0 = OK, 1 = Interrupted~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResState>))
  (cl:+ 0
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResState>))
  "Converts a ROS message object to a list"
  (cl:list 'ResState
    (cl:cons ':emergency (emergency msg))
    (cl:cons ':on_off_switch (on_off_switch msg))
    (cl:cons ':push_button (push_button msg))
    (cl:cons ':communication_interrupted (communication_interrupted msg))
))
