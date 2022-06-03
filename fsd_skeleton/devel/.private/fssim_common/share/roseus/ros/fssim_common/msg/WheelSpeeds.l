;; Auto-generated. Do not edit!


(when (boundp 'fssim_common::WheelSpeeds)
  (if (not (find-package "FSSIM_COMMON"))
    (make-package "FSSIM_COMMON"))
  (shadow 'WheelSpeeds (find-package "FSSIM_COMMON")))
(unless (find-package "FSSIM_COMMON::WHEELSPEEDS")
  (make-package "FSSIM_COMMON::WHEELSPEEDS"))

(in-package "ROS")
;;//! \htmlinclude WheelSpeeds.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass fssim_common::WheelSpeeds
  :super ros::object
  :slots (_header _version _rpm_front_left _rpm_front_right _rpm_rear_left _rpm_rear_right ))

(defmethod fssim_common::WheelSpeeds
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:version __version) 0)
    ((:rpm_front_left __rpm_front_left) 0)
    ((:rpm_front_right __rpm_front_right) 0)
    ((:rpm_rear_left __rpm_rear_left) 0)
    ((:rpm_rear_right __rpm_rear_right) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _version (round __version))
   (setq _rpm_front_left (round __rpm_front_left))
   (setq _rpm_front_right (round __rpm_front_right))
   (setq _rpm_rear_left (round __rpm_rear_left))
   (setq _rpm_rear_right (round __rpm_rear_right))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:version
   (&optional __version)
   (if __version (setq _version __version)) _version)
  (:rpm_front_left
   (&optional __rpm_front_left)
   (if __rpm_front_left (setq _rpm_front_left __rpm_front_left)) _rpm_front_left)
  (:rpm_front_right
   (&optional __rpm_front_right)
   (if __rpm_front_right (setq _rpm_front_right __rpm_front_right)) _rpm_front_right)
  (:rpm_rear_left
   (&optional __rpm_rear_left)
   (if __rpm_rear_left (setq _rpm_rear_left __rpm_rear_left)) _rpm_rear_left)
  (:rpm_rear_right
   (&optional __rpm_rear_right)
   (if __rpm_rear_right (setq _rpm_rear_right __rpm_rear_right)) _rpm_rear_right)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _version
    1
    ;; int16 _rpm_front_left
    2
    ;; int16 _rpm_front_right
    2
    ;; int16 _rpm_rear_left
    2
    ;; int16 _rpm_rear_right
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _version
       (write-byte _version s)
     ;; int16 _rpm_front_left
       (write-word _rpm_front_left s)
     ;; int16 _rpm_front_right
       (write-word _rpm_front_right s)
     ;; int16 _rpm_rear_left
       (write-word _rpm_rear_left s)
     ;; int16 _rpm_rear_right
       (write-word _rpm_rear_right s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _version
     (setq _version (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int16 _rpm_front_left
     (setq _rpm_front_left (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _rpm_front_right
     (setq _rpm_front_right (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _rpm_rear_left
     (setq _rpm_rear_left (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _rpm_rear_right
     (setq _rpm_rear_right (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get fssim_common::WheelSpeeds :md5sum-) "de6e76c895b1095f899172fc46f64a60")
(setf (get fssim_common::WheelSpeeds :datatype-) "fssim_common/WheelSpeeds")
(setf (get fssim_common::WheelSpeeds :definition-)
      "# Header
std_msgs/Header header
uint8 version

int16 rpm_front_left
int16 rpm_front_right
int16 rpm_rear_left
int16 rpm_rear_right

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :fssim_common/WheelSpeeds "de6e76c895b1095f899172fc46f64a60")


