
(cl:in-package :asdf)

(defsystem "face_tracking-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Dist" :depends-on ("_package_Dist"))
    (:file "_package_Dist" :depends-on ("_package"))
  ))