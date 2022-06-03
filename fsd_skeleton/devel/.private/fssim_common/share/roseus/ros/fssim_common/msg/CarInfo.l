;; Auto-generated. Do not edit!


(when (boundp 'fssim_common::CarInfo)
  (if (not (find-package "FSSIM_COMMON"))
    (make-package "FSSIM_COMMON"))
  (shadow 'CarInfo (find-package "FSSIM_COMMON")))
(unless (find-package "FSSIM_COMMON::CARINFO")
  (make-package "FSSIM_COMMON::CARINFO"))

(in-package "ROS")
;;//! \htmlinclude CarInfo.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass fssim_common::CarInfo
  :super ros::object
  :slots (_header _drag_force _delta _dc _front_left_steering_angle _front_right_steering_angle _delta_measured _vx _vy _r _torque_ok _alpha_f _alpha_f_l _alpha_f_r _alpha_r_l _alpha_r _alpha_r_r _Fy_f _Fy_f_l _Fy_f_r _Fy_r _Fy_r_l _Fy_r_r _Fx ))

(defmethod fssim_common::CarInfo
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:drag_force __drag_force) (instance fssim_common::Vector3Ext :init))
    ((:delta __delta) 0.0)
    ((:dc __dc) 0.0)
    ((:front_left_steering_angle __front_left_steering_angle) 0.0)
    ((:front_right_steering_angle __front_right_steering_angle) 0.0)
    ((:delta_measured __delta_measured) 0.0)
    ((:vx __vx) 0.0)
    ((:vy __vy) 0.0)
    ((:r __r) 0.0)
    ((:torque_ok __torque_ok) nil)
    ((:alpha_f __alpha_f) 0.0)
    ((:alpha_f_l __alpha_f_l) 0.0)
    ((:alpha_f_r __alpha_f_r) 0.0)
    ((:alpha_r_l __alpha_r_l) 0.0)
    ((:alpha_r __alpha_r) 0.0)
    ((:alpha_r_r __alpha_r_r) 0.0)
    ((:Fy_f __Fy_f) 0.0)
    ((:Fy_f_l __Fy_f_l) 0.0)
    ((:Fy_f_r __Fy_f_r) 0.0)
    ((:Fy_r __Fy_r) 0.0)
    ((:Fy_r_l __Fy_r_l) 0.0)
    ((:Fy_r_r __Fy_r_r) 0.0)
    ((:Fx __Fx) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _drag_force __drag_force)
   (setq _delta (float __delta))
   (setq _dc (float __dc))
   (setq _front_left_steering_angle (float __front_left_steering_angle))
   (setq _front_right_steering_angle (float __front_right_steering_angle))
   (setq _delta_measured (float __delta_measured))
   (setq _vx (float __vx))
   (setq _vy (float __vy))
   (setq _r (float __r))
   (setq _torque_ok __torque_ok)
   (setq _alpha_f (float __alpha_f))
   (setq _alpha_f_l (float __alpha_f_l))
   (setq _alpha_f_r (float __alpha_f_r))
   (setq _alpha_r_l (float __alpha_r_l))
   (setq _alpha_r (float __alpha_r))
   (setq _alpha_r_r (float __alpha_r_r))
   (setq _Fy_f (float __Fy_f))
   (setq _Fy_f_l (float __Fy_f_l))
   (setq _Fy_f_r (float __Fy_f_r))
   (setq _Fy_r (float __Fy_r))
   (setq _Fy_r_l (float __Fy_r_l))
   (setq _Fy_r_r (float __Fy_r_r))
   (setq _Fx (float __Fx))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:drag_force
   (&rest __drag_force)
   (if (keywordp (car __drag_force))
       (send* _drag_force __drag_force)
     (progn
       (if __drag_force (setq _drag_force (car __drag_force)))
       _drag_force)))
  (:delta
   (&optional __delta)
   (if __delta (setq _delta __delta)) _delta)
  (:dc
   (&optional __dc)
   (if __dc (setq _dc __dc)) _dc)
  (:front_left_steering_angle
   (&optional __front_left_steering_angle)
   (if __front_left_steering_angle (setq _front_left_steering_angle __front_left_steering_angle)) _front_left_steering_angle)
  (:front_right_steering_angle
   (&optional __front_right_steering_angle)
   (if __front_right_steering_angle (setq _front_right_steering_angle __front_right_steering_angle)) _front_right_steering_angle)
  (:delta_measured
   (&optional __delta_measured)
   (if __delta_measured (setq _delta_measured __delta_measured)) _delta_measured)
  (:vx
   (&optional __vx)
   (if __vx (setq _vx __vx)) _vx)
  (:vy
   (&optional __vy)
   (if __vy (setq _vy __vy)) _vy)
  (:r
   (&optional __r)
   (if __r (setq _r __r)) _r)
  (:torque_ok
   (&optional (__torque_ok :null))
   (if (not (eq __torque_ok :null)) (setq _torque_ok __torque_ok)) _torque_ok)
  (:alpha_f
   (&optional __alpha_f)
   (if __alpha_f (setq _alpha_f __alpha_f)) _alpha_f)
  (:alpha_f_l
   (&optional __alpha_f_l)
   (if __alpha_f_l (setq _alpha_f_l __alpha_f_l)) _alpha_f_l)
  (:alpha_f_r
   (&optional __alpha_f_r)
   (if __alpha_f_r (setq _alpha_f_r __alpha_f_r)) _alpha_f_r)
  (:alpha_r_l
   (&optional __alpha_r_l)
   (if __alpha_r_l (setq _alpha_r_l __alpha_r_l)) _alpha_r_l)
  (:alpha_r
   (&optional __alpha_r)
   (if __alpha_r (setq _alpha_r __alpha_r)) _alpha_r)
  (:alpha_r_r
   (&optional __alpha_r_r)
   (if __alpha_r_r (setq _alpha_r_r __alpha_r_r)) _alpha_r_r)
  (:Fy_f
   (&optional __Fy_f)
   (if __Fy_f (setq _Fy_f __Fy_f)) _Fy_f)
  (:Fy_f_l
   (&optional __Fy_f_l)
   (if __Fy_f_l (setq _Fy_f_l __Fy_f_l)) _Fy_f_l)
  (:Fy_f_r
   (&optional __Fy_f_r)
   (if __Fy_f_r (setq _Fy_f_r __Fy_f_r)) _Fy_f_r)
  (:Fy_r
   (&optional __Fy_r)
   (if __Fy_r (setq _Fy_r __Fy_r)) _Fy_r)
  (:Fy_r_l
   (&optional __Fy_r_l)
   (if __Fy_r_l (setq _Fy_r_l __Fy_r_l)) _Fy_r_l)
  (:Fy_r_r
   (&optional __Fy_r_r)
   (if __Fy_r_r (setq _Fy_r_r __Fy_r_r)) _Fy_r_r)
  (:Fx
   (&optional __Fx)
   (if __Fx (setq _Fx __Fx)) _Fx)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; fssim_common/Vector3Ext _drag_force
    (send _drag_force :serialization-length)
    ;; float64 _delta
    8
    ;; float64 _dc
    8
    ;; float64 _front_left_steering_angle
    8
    ;; float64 _front_right_steering_angle
    8
    ;; float64 _delta_measured
    8
    ;; float64 _vx
    8
    ;; float64 _vy
    8
    ;; float64 _r
    8
    ;; bool _torque_ok
    1
    ;; float64 _alpha_f
    8
    ;; float64 _alpha_f_l
    8
    ;; float64 _alpha_f_r
    8
    ;; float64 _alpha_r_l
    8
    ;; float64 _alpha_r
    8
    ;; float64 _alpha_r_r
    8
    ;; float64 _Fy_f
    8
    ;; float64 _Fy_f_l
    8
    ;; float64 _Fy_f_r
    8
    ;; float64 _Fy_r
    8
    ;; float64 _Fy_r_l
    8
    ;; float64 _Fy_r_r
    8
    ;; float64 _Fx
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; fssim_common/Vector3Ext _drag_force
       (send _drag_force :serialize s)
     ;; float64 _delta
       (sys::poke _delta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _dc
       (sys::poke _dc (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _front_left_steering_angle
       (sys::poke _front_left_steering_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _front_right_steering_angle
       (sys::poke _front_right_steering_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _delta_measured
       (sys::poke _delta_measured (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vx
       (sys::poke _vx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vy
       (sys::poke _vy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _r
       (sys::poke _r (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _torque_ok
       (if _torque_ok (write-byte -1 s) (write-byte 0 s))
     ;; float64 _alpha_f
       (sys::poke _alpha_f (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _alpha_f_l
       (sys::poke _alpha_f_l (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _alpha_f_r
       (sys::poke _alpha_f_r (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _alpha_r_l
       (sys::poke _alpha_r_l (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _alpha_r
       (sys::poke _alpha_r (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _alpha_r_r
       (sys::poke _alpha_r_r (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Fy_f
       (sys::poke _Fy_f (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Fy_f_l
       (sys::poke _Fy_f_l (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Fy_f_r
       (sys::poke _Fy_f_r (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Fy_r
       (sys::poke _Fy_r (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Fy_r_l
       (sys::poke _Fy_r_l (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Fy_r_r
       (sys::poke _Fy_r_r (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Fx
       (sys::poke _Fx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; fssim_common/Vector3Ext _drag_force
     (send _drag_force :deserialize buf ptr-) (incf ptr- (send _drag_force :serialization-length))
   ;; float64 _delta
     (setq _delta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _dc
     (setq _dc (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _front_left_steering_angle
     (setq _front_left_steering_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _front_right_steering_angle
     (setq _front_right_steering_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _delta_measured
     (setq _delta_measured (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vx
     (setq _vx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vy
     (setq _vy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _r
     (setq _r (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _torque_ok
     (setq _torque_ok (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _alpha_f
     (setq _alpha_f (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _alpha_f_l
     (setq _alpha_f_l (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _alpha_f_r
     (setq _alpha_f_r (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _alpha_r_l
     (setq _alpha_r_l (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _alpha_r
     (setq _alpha_r (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _alpha_r_r
     (setq _alpha_r_r (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Fy_f
     (setq _Fy_f (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Fy_f_l
     (setq _Fy_f_l (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Fy_f_r
     (setq _Fy_f_r (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Fy_r
     (setq _Fy_r (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Fy_r_l
     (setq _Fy_r_l (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Fy_r_r
     (setq _Fy_r_r (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Fx
     (setq _Fx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get fssim_common::CarInfo :md5sum-) "d1d7b96c5e9f10a89a35df8f3f330a2c")
(setf (get fssim_common::CarInfo :datatype-) "fssim_common/CarInfo")
(setf (get fssim_common::CarInfo :definition-)
      "Header header
Vector3Ext drag_force

float64 delta
float64 dc

float64 front_left_steering_angle
float64 front_right_steering_angle
float64 delta_measured

float64 vx
float64 vy
float64 r

bool torque_ok 

float64 alpha_f
float64 alpha_f_l
float64 alpha_f_r
float64 alpha_r_l
float64 alpha_r
float64 alpha_r_r

float64 Fy_f
float64 Fy_f_l
float64 Fy_f_r
float64 Fy_r
float64 Fy_r_l
float64 Fy_r_r

float64 Fx
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

================================================================================
MSG: fssim_common/Vector3Ext
geometry_msgs/Vector3 vec
float64 mag

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :fssim_common/CarInfo "d1d7b96c5e9f10a89a35df8f3f330a2c")


