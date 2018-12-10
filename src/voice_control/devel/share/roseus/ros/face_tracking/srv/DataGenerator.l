;; Auto-generated. Do not edit!


(when (boundp 'face_tracking::DataGenerator)
  (if (not (find-package "FACE_TRACKING"))
    (make-package "FACE_TRACKING"))
  (shadow 'DataGenerator (find-package "FACE_TRACKING")))
(unless (find-package "FACE_TRACKING::DATAGENERATOR")
  (make-package "FACE_TRACKING::DATAGENERATOR"))
(unless (find-package "FACE_TRACKING::DATAGENERATORREQUEST")
  (make-package "FACE_TRACKING::DATAGENERATORREQUEST"))
(unless (find-package "FACE_TRACKING::DATAGENERATORRESPONSE")
  (make-package "FACE_TRACKING::DATAGENERATORRESPONSE"))

(in-package "ROS")





(defclass face_tracking::DataGeneratorRequest
  :super ros::object
  :slots (_name ))

(defmethod face_tracking::DataGeneratorRequest
  (:init
   (&key
    ((:name __name) "")
    )
   (send-super :init)
   (setq _name (string __name))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass face_tracking::DataGeneratorResponse
  :super ros::object
  :slots (_success _message ))

(defmethod face_tracking::DataGeneratorResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass face_tracking::DataGenerator
  :super ros::object
  :slots ())

(setf (get face_tracking::DataGenerator :md5sum-) "d82dc6474dd88dad5e1615ab1b2ca74c")
(setf (get face_tracking::DataGenerator :datatype-) "face_tracking/DataGenerator")
(setf (get face_tracking::DataGenerator :request) face_tracking::DataGeneratorRequest)
(setf (get face_tracking::DataGenerator :response) face_tracking::DataGeneratorResponse)

(defmethod face_tracking::DataGeneratorRequest
  (:response () (instance face_tracking::DataGeneratorResponse :init)))

(setf (get face_tracking::DataGeneratorRequest :md5sum-) "d82dc6474dd88dad5e1615ab1b2ca74c")
(setf (get face_tracking::DataGeneratorRequest :datatype-) "face_tracking/DataGeneratorRequest")
(setf (get face_tracking::DataGeneratorRequest :definition-)
      "string name
---
bool success
string message
")

(setf (get face_tracking::DataGeneratorResponse :md5sum-) "d82dc6474dd88dad5e1615ab1b2ca74c")
(setf (get face_tracking::DataGeneratorResponse :datatype-) "face_tracking/DataGeneratorResponse")
(setf (get face_tracking::DataGeneratorResponse :definition-)
      "string name
---
bool success
string message
")



(provide :face_tracking/DataGenerator "d82dc6474dd88dad5e1615ab1b2ca74c")


