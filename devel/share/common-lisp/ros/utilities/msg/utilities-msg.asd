
(cl:in-package :asdf)

(defsystem "utilities-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "gripperTestRequest" :depends-on ("_package_gripperTestRequest"))
    (:file "_package_gripperTestRequest" :depends-on ("_package"))
    (:file "gripperTestStatus" :depends-on ("_package_gripperTestStatus"))
    (:file "_package_gripperTestStatus" :depends-on ("_package"))
  ))