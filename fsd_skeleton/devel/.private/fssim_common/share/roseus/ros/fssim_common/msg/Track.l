;; Auto-generated. Do not edit!


(when (boundp 'fssim_common::Track)
  (if (not (find-package "FSSIM_COMMON"))
    (make-package "FSSIM_COMMON"))
  (shadow 'Track (find-package "FSSIM_COMMON")))
(unless (find-package "FSSIM_COMMON::TRACK")
  (make-package "FSSIM_COMMON::TRACK"))

(in-package "ROS")
;;//! \htmlinclude Track.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass fssim_common::Track
  :super ros::object
  :slots (_header _cones_left _cones_right _cones_orange _cones_orange_big _tk_device_start _tk_device_end ))

(defmethod fssim_common::Track
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:cones_left __cones_left) ())
    ((:cones_right __cones_right) ())
    ((:cones_orange __cones_orange) ())
    ((:cones_orange_big __cones_orange_big) ())
    ((:tk_device_start __tk_device_start) ())
    ((:tk_device_end __tk_device_end) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _cones_left __cones_left)
   (setq _cones_right __cones_right)
   (setq _cones_orange __cones_orange)
   (setq _cones_orange_big __cones_orange_big)
   (setq _tk_device_start __tk_device_start)
   (setq _tk_device_end __tk_device_end)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:cones_left
   (&rest __cones_left)
   (if (keywordp (car __cones_left))
       (send* _cones_left __cones_left)
     (progn
       (if __cones_left (setq _cones_left (car __cones_left)))
       _cones_left)))
  (:cones_right
   (&rest __cones_right)
   (if (keywordp (car __cones_right))
       (send* _cones_right __cones_right)
     (progn
       (if __cones_right (setq _cones_right (car __cones_right)))
       _cones_right)))
  (:cones_orange
   (&rest __cones_orange)
   (if (keywordp (car __cones_orange))
       (send* _cones_orange __cones_orange)
     (progn
       (if __cones_orange (setq _cones_orange (car __cones_orange)))
       _cones_orange)))
  (:cones_orange_big
   (&rest __cones_orange_big)
   (if (keywordp (car __cones_orange_big))
       (send* _cones_orange_big __cones_orange_big)
     (progn
       (if __cones_orange_big (setq _cones_orange_big (car __cones_orange_big)))
       _cones_orange_big)))
  (:tk_device_start
   (&rest __tk_device_start)
   (if (keywordp (car __tk_device_start))
       (send* _tk_device_start __tk_device_start)
     (progn
       (if __tk_device_start (setq _tk_device_start (car __tk_device_start)))
       _tk_device_start)))
  (:tk_device_end
   (&rest __tk_device_end)
   (if (keywordp (car __tk_device_end))
       (send* _tk_device_end __tk_device_end)
     (progn
       (if __tk_device_end (setq _tk_device_end (car __tk_device_end)))
       _tk_device_end)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Point[] _cones_left
    (apply #'+ (send-all _cones_left :serialization-length)) 4
    ;; geometry_msgs/Point[] _cones_right
    (apply #'+ (send-all _cones_right :serialization-length)) 4
    ;; geometry_msgs/Point[] _cones_orange
    (apply #'+ (send-all _cones_orange :serialization-length)) 4
    ;; geometry_msgs/Point[] _cones_orange_big
    (apply #'+ (send-all _cones_orange_big :serialization-length)) 4
    ;; geometry_msgs/Point[] _tk_device_start
    (apply #'+ (send-all _tk_device_start :serialization-length)) 4
    ;; geometry_msgs/Point[] _tk_device_end
    (apply #'+ (send-all _tk_device_end :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Point[] _cones_left
     (write-long (length _cones_left) s)
     (dolist (elem _cones_left)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[] _cones_right
     (write-long (length _cones_right) s)
     (dolist (elem _cones_right)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[] _cones_orange
     (write-long (length _cones_orange) s)
     (dolist (elem _cones_orange)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[] _cones_orange_big
     (write-long (length _cones_orange_big) s)
     (dolist (elem _cones_orange_big)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[] _tk_device_start
     (write-long (length _tk_device_start) s)
     (dolist (elem _tk_device_start)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[] _tk_device_end
     (write-long (length _tk_device_end) s)
     (dolist (elem _tk_device_end)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Point[] _cones_left
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cones_left (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _cones_left)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _cones_right
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cones_right (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _cones_right)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _cones_orange
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cones_orange (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _cones_orange)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _cones_orange_big
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cones_orange_big (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _cones_orange_big)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _tk_device_start
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tk_device_start (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _tk_device_start)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _tk_device_end
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tk_device_end (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _tk_device_end)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get fssim_common::Track :md5sum-) "b3e47dce05b0e0c04dc61079e1408cdf")
(setf (get fssim_common::Track :datatype-) "fssim_common/Track")
(setf (get fssim_common::Track :definition-)
      "std_msgs/Header header
geometry_msgs/Point[] cones_left
geometry_msgs/Point[] cones_right
geometry_msgs/Point[] cones_orange
geometry_msgs/Point[] cones_orange_big
geometry_msgs/Point[] tk_device_start
geometry_msgs/Point[] tk_device_end
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
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :fssim_common/Track "b3e47dce05b0e0c04dc61079e1408cdf")


