;; Auto-generated. Do not edit!


(when (boundp 'face_tracking::DataTrainer)
  (if (not (find-package "FACE_TRACKING"))
    (make-package "FACE_TRACKING"))
  (shadow 'DataTrainer (find-package "FACE_TRACKING")))
(unless (find-package "FACE_TRACKING::DATATRAINER")
  (make-package "FACE_TRACKING::DATATRAINER"))
(unless (find-package "FACE_TRACKING::DATATRAINERREQUEST")
  (make-package "FACE_TRACKING::DATATRAINERREQUEST"))
(unless (find-package "FACE_TRACKING::DATATRAINERRESPONSE")
  (make-package "FACE_TRACKING::DATATRAINERRESPONSE"))

(in-package "ROS")





(defclass face_tracking::DataTrainerRequest
  :super ros::object
  :slots (_onoff ))

(defmethod face_tracking::DataTrainerRequest
  (:init
   (&key
    ((:onoff __onoff) nil)
    )
   (send-super :init)
   (setq _onoff __onoff)
   self)
  (:onoff
   (&optional __onoff)
   (if __onoff (setq _onoff __onoff)) _onoff)
  (:serialization-length
   ()
   (+
    ;; bool _onoff
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _onoff
       (if _onoff (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _onoff
     (setq _onoff (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass face_tracking::DataTrainerResponse
  :super ros::object
  :slots (_success _num ))

(defmethod face_tracking::DataTrainerResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:num __num) 0)
    )
   (send-super :init)
   (setq _success __success)
   (setq _num (round __num))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:num
   (&optional __num)
   (if __num (setq _num __num)) _num)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; int64 _num
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; int64 _num
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _num (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _num) (= (length (_num . bv)) 2)) ;; bignum
              (write-long (ash (elt (_num . bv) 0) 0) s)
              (write-long (ash (elt (_num . bv) 1) -1) s))
             ((and (class _num) (= (length (_num . bv)) 1)) ;; big1
              (write-long (elt (_num . bv) 0) s)
              (write-long (if (>= _num 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _num s)(write-long (if (>= _num 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int64 _num
#+(or :alpha :irix6 :x86_64)
      (setf _num (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _num (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass face_tracking::DataTrainer
  :super ros::object
  :slots ())

(setf (get face_tracking::DataTrainer :md5sum-) "07f48dc3148a93e6660a9d3c75b3c6af")
(setf (get face_tracking::DataTrainer :datatype-) "face_tracking/DataTrainer")
(setf (get face_tracking::DataTrainer :request) face_tracking::DataTrainerRequest)
(setf (get face_tracking::DataTrainer :response) face_tracking::DataTrainerResponse)

(defmethod face_tracking::DataTrainerRequest
  (:response () (instance face_tracking::DataTrainerResponse :init)))

(setf (get face_tracking::DataTrainerRequest :md5sum-) "07f48dc3148a93e6660a9d3c75b3c6af")
(setf (get face_tracking::DataTrainerRequest :datatype-) "face_tracking/DataTrainerRequest")
(setf (get face_tracking::DataTrainerRequest :definition-)
      "bool onoff
---
bool success
int64 num

")

(setf (get face_tracking::DataTrainerResponse :md5sum-) "07f48dc3148a93e6660a9d3c75b3c6af")
(setf (get face_tracking::DataTrainerResponse :datatype-) "face_tracking/DataTrainerResponse")
(setf (get face_tracking::DataTrainerResponse :definition-)
      "bool onoff
---
bool success
int64 num

")



(provide :face_tracking/DataTrainer "07f48dc3148a93e6660a9d3c75b3c6af")


