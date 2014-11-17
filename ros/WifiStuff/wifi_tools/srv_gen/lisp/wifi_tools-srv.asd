
(cl:in-package :asdf)

(defsystem "wifi_tools-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Out2File" :depends-on ("_package_Out2File"))
    (:file "_package_Out2File" :depends-on ("_package"))
  ))