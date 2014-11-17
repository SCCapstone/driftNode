; Auto-generated. Do not edit!


(cl:in-package wifi_tools-msg)


;//! \htmlinclude AccessPoint.msg.html

(cl:defclass <AccessPoint> (roslisp-msg-protocol:ros-message)
  ((mac_address
    :reader mac_address
    :initarg :mac_address
    :type cl:string
    :initform "")
   (ss
    :reader ss
    :initarg :ss
    :type cl:fixnum
    :initform 0)
   (noise
    :reader noise
    :initarg :noise
    :type cl:fixnum
    :initform 0))
)

(cl:defclass AccessPoint (<AccessPoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AccessPoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AccessPoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wifi_tools-msg:<AccessPoint> is deprecated: use wifi_tools-msg:AccessPoint instead.")))

(cl:ensure-generic-function 'mac_address-val :lambda-list '(m))
(cl:defmethod mac_address-val ((m <AccessPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wifi_tools-msg:mac_address-val is deprecated.  Use wifi_tools-msg:mac_address instead.")
  (mac_address m))

(cl:ensure-generic-function 'ss-val :lambda-list '(m))
(cl:defmethod ss-val ((m <AccessPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wifi_tools-msg:ss-val is deprecated.  Use wifi_tools-msg:ss instead.")
  (ss m))

(cl:ensure-generic-function 'noise-val :lambda-list '(m))
(cl:defmethod noise-val ((m <AccessPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wifi_tools-msg:noise-val is deprecated.  Use wifi_tools-msg:noise instead.")
  (noise m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AccessPoint>) ostream)
  "Serializes a message object of type '<AccessPoint>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mac_address))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mac_address))
  (cl:let* ((signed (cl:slot-value msg 'ss)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'noise)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AccessPoint>) istream)
  "Deserializes a message object of type '<AccessPoint>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mac_address) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mac_address) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ss) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'noise) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AccessPoint>)))
  "Returns string type for a message object of type '<AccessPoint>"
  "wifi_tools/AccessPoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AccessPoint)))
  "Returns string type for a message object of type 'AccessPoint"
  "wifi_tools/AccessPoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AccessPoint>)))
  "Returns md5sum for a message object of type '<AccessPoint>"
  "14566870ce27da9eaacf517d23cded21")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AccessPoint)))
  "Returns md5sum for a message object of type 'AccessPoint"
  "14566870ce27da9eaacf517d23cded21")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AccessPoint>)))
  "Returns full string definition for message of type '<AccessPoint>"
  (cl:format cl:nil "string mac_address # MAC address of an access point~%int16 ss           # signal strength [RSSI]~%int16 noise        # noise~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AccessPoint)))
  "Returns full string definition for message of type 'AccessPoint"
  (cl:format cl:nil "string mac_address # MAC address of an access point~%int16 ss           # signal strength [RSSI]~%int16 noise        # noise~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AccessPoint>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'mac_address))
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AccessPoint>))
  "Converts a ROS message object to a list"
  (cl:list 'AccessPoint
    (cl:cons ':mac_address (mac_address msg))
    (cl:cons ':ss (ss msg))
    (cl:cons ':noise (noise msg))
))
