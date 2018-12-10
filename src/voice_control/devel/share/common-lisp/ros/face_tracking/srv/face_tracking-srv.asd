
(cl:in-package :asdf)

(defsystem "face_tracking-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "DataGenerator" :depends-on ("_package_DataGenerator"))
    (:file "_package_DataGenerator" :depends-on ("_package"))
    (:file "DataTrainer" :depends-on ("_package_DataTrainer"))
    (:file "_package_DataTrainer" :depends-on ("_package"))
  ))