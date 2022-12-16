
(cl:in-package :asdf)

(defsystem "move_arm-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "target" :depends-on ("_package_target"))
    (:file "_package_target" :depends-on ("_package"))
  ))