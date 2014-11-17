; Auto-generated. Do not edit!


(cl:in-package wifi_tools-msg)


;//! \htmlinclude WifiData.msg.html

(cl:defclass <WifiData> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (data
    :reader data
    :initarg :data
    :type (cl:vector wifi_tools-msg:AccessPoint)
   :initform (cl:make-array 0 :element-type 'wifi_tools-msg:AccessPoint :initial-element (cl:make-instance 'wifi_tools-msg:AccessPoint))))
)

(cl:defclass WifiData (<WifiData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WifiData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WifiData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wifi_tools-msg:<WifiData> is deprecated: use wifi_tools-msg:WifiData instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <WifiData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wifi_tools-msg:header-val is deprecated.  Use wifi_tools-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <WifiData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wifi_tools-msg:data-val is deprecated.  Use wifi_tools-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WifiData>) ostream)
  "Serializes a message object of type '<WifiData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WifiData>) istream)
  "Deserializes a message object of type '<WifiData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'wifi_tools-msg:AccessPoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WifiData>)))
  "Returns string type for a message object of type '<WifiData>"
  "wifi_tools/WifiData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WifiData)))
  "Returns string type for a message object of type 'WifiData"
  "wifi_tools/WifiData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WifiData>)))
  "Returns md5sum for a message object of type '<WifiData>"
  "e1957fecb2a1644dd1be38d5c3a730a4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WifiData)))
  "Returns md5sum for a message object of type 'WifiData"
  "e1957fecb2a1644dd1be38d5c3a730a4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WifiData>)))
  "Returns full string definition for message of type '<WifiData>"
  (cl:format cl:nil "Header header       # timestamp~%AccessPoint[] data  # wifi data of all access points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: wifi_tools/AccessPoint~%string mac_address # MAC address of an access point~%int16 ss           # signal strength [RSSI]~%int16 noise        # noise~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WifiData)))
  "Returns full string definition for message of type 'WifiData"
  (cl:format cl:nil "Header header       # timestamp~%AccessPoint[] data  # wifi data of all access points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: wifi_tools/AccessPoint~%string mac_address # MAC address of an access point~%int16 ss           # signal strength [RSSI]~%int16 noise        # noise~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WifiData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WifiData>))
  "Converts a ROS message object to a list"
  (cl:list 'WifiData
    (cl:cons ':header (header msg))
    (cl:cons ':data (data msg))
))
