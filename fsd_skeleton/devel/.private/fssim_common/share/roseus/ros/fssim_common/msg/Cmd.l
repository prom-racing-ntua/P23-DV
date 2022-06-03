;; Auto-generated. Do not edit!


(when (boundp 'fssim_common::Cmd)
  (if (not (find-package "FSSIM_COMMON"))
    (make-package "FSSIM_COMMON"))
  (shadow 'Cmd (find-package "FSSIM_COMMON")))
(unless (find-package "FSSIM_COMMON::CMD")
  (make-package "FSSIM_COMMON::CMD"))

(in-package "ROS")
;;//! \htmlinclude Cmd.msg.html


(defclass fssim_common::Cmd
  :super ros::object
  :slots (_dc _delta ))

(defmethod fssim_common::Cmd
  (:init
   (&key
    ((:dc __dc) 0.0)
    ((:delta __delta) 0.0)
    )
   (send-super :init)
   (setq _dc (float __dc))
   (setq _delta (float __delta))
   self)
  (:dc
   (&optional __dc)
   (if __dc (setq _dc __dc)) _dc)
  (:delta
   (&optional __delta)
   (if __delta (setq _delta __delta)) _delta)
  (:serialization-length
   ()
   (+
    ;; float64 _dc
    8
    ;; float64 _delta
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _dc
       (sys::poke _dc (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _delta
       (sys::poke _delta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _dc
     (setq _dc (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _delta
     (setq _delta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get fssim_common::Cmd :md5sum-) "7c30c8d10cd397c67459b00d587e06f1")
(setf (get fssim_common::Cmd :datatype-) "fssim_common/Cmd")
(setf (get fssim_common::Cmd :definition-)
      "float64 dc
float64 delta


")



(provide :fssim_common/Cmd "7c30c8d10cd397c67459b00d587e06f1")


