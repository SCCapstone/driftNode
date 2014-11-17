
(cl:in-package :asdf)

(defsystem "wifi_tools-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "WifiData" :depends-on ("_package_WifiData"))
    (:file "_package_WifiData" :depends-on ("_package"))
    (:file "AccessPoint" :depends-on ("_package_AccessPoint"))
    (:file "_package_AccessPoint" :depends-on ("_package"))
  ))