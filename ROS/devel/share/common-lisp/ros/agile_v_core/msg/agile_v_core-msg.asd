
(cl:in-package :asdf)

(defsystem "agile_v_core-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "electric" :depends-on ("_package_electric"))
    (:file "_package_electric" :depends-on ("_package"))
    (:file "joyinfoex" :depends-on ("_package_joyinfoex"))
    (:file "_package_joyinfoex" :depends-on ("_package"))
    (:file "kinematics" :depends-on ("_package_kinematics"))
    (:file "_package_kinematics" :depends-on ("_package"))
  ))