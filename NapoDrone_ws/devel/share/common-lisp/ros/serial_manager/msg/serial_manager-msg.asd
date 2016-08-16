
(cl:in-package :asdf)

(defsystem "serial_manager-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Param" :depends-on ("_package_Param"))
    (:file "_package_Param" :depends-on ("_package"))
  ))