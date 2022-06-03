;; Auto-generated. Do not edit!


(when (boundp 'fssim_common::Vector3Ext)
  (if (not (find-package "FSSIM_COMMON"))
    (make-package "FSSIM_COMMON"))
  (shadow 'Vector3Ext (find-package "FSSIM_COMMON")))
(unless (find-package "FSSIM_COMMON::VECTOR3EXT")
  (make-package "FSSIM_COMMON::VECTOR3EXT"))

(in-package "ROS")
;;//! \htmlinclude Vector3Ext.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass fssim_common::Vector3Ext
  :super ros::object
  :slots (_vec _mag ))

(defmethod fssim_common::Vector3Ext
  (:init
   (&key
    ((:vec __vec) (instance geometry_msgs::Vector3 :init))
    ((:mag __mag) 0.0)
    )
   (send-super :init)
   (setq _vec __vec)
   (setq _mag (float __mag))
   self)
  (:vec
   (&rest __vec)
   (if (keywordp (car __vec))
       (send* _vec __vec)
     (progn
       (if __vec (setq _vec (car __vec)))
       _vec)))
  (:mag
   (&optional __mag)
   (if __mag (setq _mag __mag)) _mag)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Vector3 _vec
    (send _vec :serialization-length)
    ;; float64 _mag
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Vector3 _vec
       (send _vec :serialize s)
     ;; float64 _mag
       (sys::poke _mag (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Vector3 _vec
     (send _vec :deserialize buf ptr-) (incf ptr- (send _vec :serialization-length))
   ;; float64 _mag
     (setq _mag (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get fssim_common::Vector3Ext :md5sum-) "9ce7c2ac065b8145af6ba1b53af0b0bd")
(setf (get fssim_common::Vector3Ext :datatype-) "fssim_common/Vector3Ext")
(setf (get fssim_common::Vector3Ext :definition-)
      "geometry_msgs/Vector3 vec
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



(provide :fssim_common/Vector3Ext "9ce7c2ac065b8145af6ba1b53af0b0bd")


