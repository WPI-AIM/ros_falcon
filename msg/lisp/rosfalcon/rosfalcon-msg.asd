
(in-package :asdf)

(defsystem "rosfalcon-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "falconSetPoint" :depends-on ("_package"))
    (:file "_package_falconSetPoint" :depends-on ("_package"))
    (:file "falconPos" :depends-on ("_package"))
    (:file "_package_falconPos" :depends-on ("_package"))
    (:file "falconForces" :depends-on ("_package"))
    (:file "_package_falconForces" :depends-on ("_package"))
    ))
