;; Auto-generated. Do not edit!


(when (boundp 'speak_tools::SpeakedText)
  (if (not (find-package "SPEAK_TOOLS"))
    (make-package "SPEAK_TOOLS"))
  (shadow 'SpeakedText (find-package "SPEAK_TOOLS")))
(unless (find-package "SPEAK_TOOLS::SPEAKEDTEXT")
  (make-package "SPEAK_TOOLS::SPEAKEDTEXT"))
(unless (find-package "SPEAK_TOOLS::SPEAKEDTEXTREQUEST")
  (make-package "SPEAK_TOOLS::SPEAKEDTEXTREQUEST"))
(unless (find-package "SPEAK_TOOLS::SPEAKEDTEXTRESPONSE")
  (make-package "SPEAK_TOOLS::SPEAKEDTEXTRESPONSE"))

(in-package "ROS")





(defclass speak_tools::SpeakedTextRequest
  :super ros::object
  :slots (_message ))

(defmethod speak_tools::SpeakedTextRequest
  (:init
   (&key
    ((:message __message) "")
    )
   (send-super :init)
   (setq _message (string __message))
   self)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass speak_tools::SpeakedTextResponse
  :super ros::object
  :slots (_success ))

(defmethod speak_tools::SpeakedTextResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass speak_tools::SpeakedText
  :super ros::object
  :slots ())

(setf (get speak_tools::SpeakedText :md5sum-) "8b7095eb8dcd517ba7c37a0a06dcc50b")
(setf (get speak_tools::SpeakedText :datatype-) "speak_tools/SpeakedText")
(setf (get speak_tools::SpeakedText :request) speak_tools::SpeakedTextRequest)
(setf (get speak_tools::SpeakedText :response) speak_tools::SpeakedTextResponse)

(defmethod speak_tools::SpeakedTextRequest
  (:response () (instance speak_tools::SpeakedTextResponse :init)))

(setf (get speak_tools::SpeakedTextRequest :md5sum-) "8b7095eb8dcd517ba7c37a0a06dcc50b")
(setf (get speak_tools::SpeakedTextRequest :datatype-) "speak_tools/SpeakedTextRequest")
(setf (get speak_tools::SpeakedTextRequest :definition-)
      "string message
---
bool success
")

(setf (get speak_tools::SpeakedTextResponse :md5sum-) "8b7095eb8dcd517ba7c37a0a06dcc50b")
(setf (get speak_tools::SpeakedTextResponse :datatype-) "speak_tools/SpeakedTextResponse")
(setf (get speak_tools::SpeakedTextResponse :definition-)
      "string message
---
bool success
")



(provide :speak_tools/SpeakedText "8b7095eb8dcd517ba7c37a0a06dcc50b")


