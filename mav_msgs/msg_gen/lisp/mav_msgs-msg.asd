
(cl:in-package :asdf)

(defsystem "mav_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Height" :depends-on ("_package_Height"))
    (:file "_package_Height" :depends-on ("_package"))
    (:file "State" :depends-on ("_package_State"))
    (:file "_package_State" :depends-on ("_package"))
  ))