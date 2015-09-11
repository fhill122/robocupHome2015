
(cl:in-package :asdf)

(defsystem "vision-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "platePosition" :depends-on ("_package_platePosition"))
    (:file "_package_platePosition" :depends-on ("_package"))
  ))