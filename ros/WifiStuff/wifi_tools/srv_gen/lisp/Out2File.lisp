; Auto-generated. Do not edit!


(cl:in-package wifi_tools-srv)


;//! \htmlinclude Out2File-request.msg.html

(cl:defclass <Out2File-request> (roslisp-msg-protocol:ros-message)
  ((req
    :reader req
    :initarg :req
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Out2File-request (<Out2File-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Out2File-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Out2File-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wifi_tools-srv:<Out2File-request> is deprecated: use wifi_tools-srv:Out2File-request instead.")))

(cl:ensure-generic-function 'req-val :lambda-list '(m))
(cl:defmethod req-val ((m <Out2File-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wifi_tools-srv:req-val is deprecated.  Use wifi_tools-srv:req instead.")
  (req m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Out2File-request>) ostream)
  "Serializes a message object of type '<Out2File-request>"
  (cl:let* ((signed (cl:slot-value msg 'req)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Out2File-request>) istream)
  "Deserializes a message object of type '<Out2File-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'req) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Out2File-request>)))
  "Returns string type for a service object of type '<Out2File-request>"
  "wifi_tools/Out2FileRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Out2File-request)))
  "Returns string type for a service object of type 'Out2File-request"
  "wifi_tools/Out2FileRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Out2File-request>)))
  "Returns md5sum for a message object of type '<Out2File-request>"
  "a1fed1837cc4ac5b3d3edf0194c323e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Out2File-request)))
  "Returns md5sum for a message object of type 'Out2File-request"
  "a1fed1837cc4ac5b3d3edf0194c323e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Out2File-request>)))
  "Returns full string definition for message of type '<Out2File-request>"
  (cl:format cl:nil "int8 req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Out2File-request)))
  "Returns full string definition for message of type 'Out2File-request"
  (cl:format cl:nil "int8 req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Out2File-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Out2File-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Out2File-request
    (cl:cons ':req (req msg))
))
;//! \htmlinclude Out2File-response.msg.html

(cl:defclass <Out2File-response> (roslisp-msg-protocol:ros-message)
  ((res
    :reader res
    :initarg :res
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Out2File-response (<Out2File-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Out2File-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Out2File-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name wifi_tools-srv:<Out2File-response> is deprecated: use wifi_tools-srv:Out2File-response instead.")))

(cl:ensure-generic-function 'res-val :lambda-list '(m))
(cl:defmethod res-val ((m <Out2File-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader wifi_tools-srv:res-val is deprecated.  Use wifi_tools-srv:res instead.")
  (res m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Out2File-response>) ostream)
  "Serializes a message object of type '<Out2File-response>"
  (cl:let* ((signed (cl:slot-value msg 'res)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Out2File-response>) istream)
  "Deserializes a message object of type '<Out2File-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'res) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Out2File-response>)))
  "Returns string type for a service object of type '<Out2File-response>"
  "wifi_tools/Out2FileResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Out2File-response)))
  "Returns string type for a service object of type 'Out2File-response"
  "wifi_tools/Out2FileResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Out2File-response>)))
  "Returns md5sum for a message object of type '<Out2File-response>"
  "a1fed1837cc4ac5b3d3edf0194c323e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Out2File-response)))
  "Returns md5sum for a message object of type 'Out2File-response"
  "a1fed1837cc4ac5b3d3edf0194c323e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Out2File-response>)))
  "Returns full string definition for message of type '<Out2File-response>"
  (cl:format cl:nil "int8 res~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Out2File-response)))
  "Returns full string definition for message of type 'Out2File-response"
  (cl:format cl:nil "int8 res~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Out2File-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Out2File-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Out2File-response
    (cl:cons ':res (res msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Out2File)))
  'Out2File-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Out2File)))
  'Out2File-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Out2File)))
  "Returns string type for a service object of type '<Out2File>"
  "wifi_tools/Out2File")