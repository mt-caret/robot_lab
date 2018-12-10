;; Auto-generated. Do not edit!


(when (boundp 'face_tracking::Dist)
  (if (not (find-package "FACE_TRACKING"))
    (make-package "FACE_TRACKING"))
  (shadow 'Dist (find-package "FACE_TRACKING")))
(unless (find-package "FACE_TRACKING::DIST")
  (make-package "FACE_TRACKING::DIST"))

(in-package "ROS")
;;//! \htmlinclude Dist.msg.html


(defclass face_tracking::Dist
  :super ros::object
  :slots (_anglar _dist ))

(defmethod face_tracking::Dist
  (:init
   (&key
    ((:anglar __anglar) 0.0)
    ((:dist __dist) 0)
    )
   (send-super :init)
   (setq _anglar (float __anglar))
   (setq _dist (round __dist))
   self)
  (:anglar
   (&optional __anglar)
   (if __anglar (setq _anglar __anglar)) _anglar)
  (:dist
   (&optional __dist)
   (if __dist (setq _dist __dist)) _dist)
  (:serialization-length
   ()
   (+
    ;; float32 _anglar
    4
    ;; int32 _dist
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _anglar
       (sys::poke _anglar (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _dist
       (write-long _dist s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _anglar
     (setq _anglar (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _dist
     (setq _dist (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get face_tracking::Dist :md5sum-) "2217c177b412cc5ac6a43674c38fcf6a")
(setf (get face_tracking::Dist :datatype-) "face_tracking/Dist")
(setf (get face_tracking::Dist :definition-)
      "float32 anglar
int32 dist
")



(provide :face_tracking/Dist "2217c177b412cc5ac6a43674c38fcf6a")


