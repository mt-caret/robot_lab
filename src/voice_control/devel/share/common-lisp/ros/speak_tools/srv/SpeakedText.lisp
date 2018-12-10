; Auto-generated. Do not edit!


(cl:in-package speak_tools-srv)


;//! \htmlinclude SpeakedText-request.msg.html

(cl:defclass <SpeakedText-request> (roslisp-msg-protocol:ros-message)
  ((message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass SpeakedText-request (<SpeakedText-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpeakedText-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpeakedText-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name speak_tools-srv:<SpeakedText-request> is deprecated: use speak_tools-srv:SpeakedText-request instead.")))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SpeakedText-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speak_tools-srv:message-val is deprecated.  Use speak_tools-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpeakedText-request>) ostream)
  "Serializes a message object of type '<SpeakedText-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpeakedText-request>) istream)
  "Deserializes a message object of type '<SpeakedText-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpeakedText-request>)))
  "Returns string type for a service object of type '<SpeakedText-request>"
  "speak_tools/SpeakedTextRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpeakedText-request)))
  "Returns string type for a service object of type 'SpeakedText-request"
  "speak_tools/SpeakedTextRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpeakedText-request>)))
  "Returns md5sum for a message object of type '<SpeakedText-request>"
  "8b7095eb8dcd517ba7c37a0a06dcc50b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpeakedText-request)))
  "Returns md5sum for a message object of type 'SpeakedText-request"
  "8b7095eb8dcd517ba7c37a0a06dcc50b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpeakedText-request>)))
  "Returns full string definition for message of type '<SpeakedText-request>"
  (cl:format cl:nil "string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpeakedText-request)))
  "Returns full string definition for message of type 'SpeakedText-request"
  (cl:format cl:nil "string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpeakedText-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpeakedText-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SpeakedText-request
    (cl:cons ':message (message msg))
))
;//! \htmlinclude SpeakedText-response.msg.html

(cl:defclass <SpeakedText-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SpeakedText-response (<SpeakedText-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpeakedText-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpeakedText-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name speak_tools-srv:<SpeakedText-response> is deprecated: use speak_tools-srv:SpeakedText-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SpeakedText-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader speak_tools-srv:success-val is deprecated.  Use speak_tools-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpeakedText-response>) ostream)
  "Serializes a message object of type '<SpeakedText-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpeakedText-response>) istream)
  "Deserializes a message object of type '<SpeakedText-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpeakedText-response>)))
  "Returns string type for a service object of type '<SpeakedText-response>"
  "speak_tools/SpeakedTextResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpeakedText-response)))
  "Returns string type for a service object of type 'SpeakedText-response"
  "speak_tools/SpeakedTextResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpeakedText-response>)))
  "Returns md5sum for a message object of type '<SpeakedText-response>"
  "8b7095eb8dcd517ba7c37a0a06dcc50b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpeakedText-response)))
  "Returns md5sum for a message object of type 'SpeakedText-response"
  "8b7095eb8dcd517ba7c37a0a06dcc50b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpeakedText-response>)))
  "Returns full string definition for message of type '<SpeakedText-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpeakedText-response)))
  "Returns full string definition for message of type 'SpeakedText-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpeakedText-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpeakedText-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SpeakedText-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SpeakedText)))
  'SpeakedText-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SpeakedText)))
  'SpeakedText-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpeakedText)))
  "Returns string type for a service object of type '<SpeakedText>"
  "speak_tools/SpeakedText")