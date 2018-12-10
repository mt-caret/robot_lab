
(cl:in-package :asdf)

(defsystem "speak_tools-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SpeakedText" :depends-on ("_package_SpeakedText"))
    (:file "_package_SpeakedText" :depends-on ("_package"))
  ))