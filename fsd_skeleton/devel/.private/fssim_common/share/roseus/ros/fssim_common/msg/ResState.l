;; Auto-generated. Do not edit!


(when (boundp 'fssim_common::ResState)
  (if (not (find-package "FSSIM_COMMON"))
    (make-package "FSSIM_COMMON"))
  (shadow 'ResState (find-package "FSSIM_COMMON")))
(unless (find-package "FSSIM_COMMON::RESSTATE")
  (make-package "FSSIM_COMMON::RESSTATE"))

(in-package "ROS")
;;//! \htmlinclude ResState.msg.html


(defclass fssim_common::ResState
  :super ros::object
  :slots (_emergency _on_off_switch _push_button _communication_interrupted ))

(defmethod fssim_common::ResState
  (:init
   (&key
    ((:emergency __emergency) nil)
    ((:on_off_switch __on_off_switch) nil)
    ((:push_button __push_button) nil)
    ((:communication_interrupted __communication_interrupted) nil)
    )
   (send-super :init)
   (setq _emergency __emergency)
   (setq _on_off_switch __on_off_switch)
   (setq _push_button __push_button)
   (setq _communication_interrupted __communication_interrupted)
   self)
  (:emergency
   (&optional (__emergency :null))
   (if (not (eq __emergency :null)) (setq _emergency __emergency)) _emergency)
  (:on_off_switch
   (&optional (__on_off_switch :null))
   (if (not (eq __on_off_switch :null)) (setq _on_off_switch __on_off_switch)) _on_off_switch)
  (:push_button
   (&optional (__push_button :null))
   (if (not (eq __push_button :null)) (setq _push_button __push_button)) _push_button)
  (:communication_interrupted
   (&optional (__communication_interrupted :null))
   (if (not (eq __communication_interrupted :null)) (setq _communication_interrupted __communication_interrupted)) _communication_interrupted)
  (:serialization-length
   ()
   (+
    ;; bool _emergency
    1
    ;; bool _on_off_switch
    1
    ;; bool _push_button
    1
    ;; bool _communication_interrupted
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _emergency
       (if _emergency (write-byte -1 s) (write-byte 0 s))
     ;; bool _on_off_switch
       (if _on_off_switch (write-byte -1 s) (write-byte 0 s))
     ;; bool _push_button
       (if _push_button (write-byte -1 s) (write-byte 0 s))
     ;; bool _communication_interrupted
       (if _communication_interrupted (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _emergency
     (setq _emergency (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _on_off_switch
     (setq _on_off_switch (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _push_button
     (setq _push_button (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _communication_interrupted
     (setq _communication_interrupted (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get fssim_common::ResState :md5sum-) "2c68d942044efe0714c25879acd65327")
(setf (get fssim_common::ResState :datatype-) "fssim_common/ResState")
(setf (get fssim_common::ResState :definition-)
      "# State of the three buttons on the RES
bool emergency 					# 0 = OK, 1 = Emergency
bool on_off_switch 				# 0 = 0, 1 = 1 (Comments couldn't be more helpful :p )
bool push_button 				# 1 = pressed
# RES will trigger emergency 200ms after this becomes 1 (unless communication is recovered)
bool communication_interrupted 	# 0 = OK, 1 = Interrupted

")



(provide :fssim_common/ResState "2c68d942044efe0714c25879acd65327")


