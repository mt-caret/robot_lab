; Auto-generated. Do not edit!


(cl:in-package face_tracking-srv)


;//! \htmlinclude DataGenerator-request.msg.html

(cl:defclass <DataGenerator-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass DataGenerator-request (<DataGenerator-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DataGenerator-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DataGenerator-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name face_tracking-srv:<DataGenerator-request> is deprecated: use face_tracking-srv:DataGenerator-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <DataGenerator-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader face_tracking-srv:name-val is deprecated.  Use face_tracking-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DataGenerator-request>) ostream)
  "Serializes a message object of type '<DataGenerator-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DataGenerator-request>) istream)
  "Deserializes a message object of type '<DataGenerator-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DataGenerator-request>)))
  "Returns string type for a service object of type '<DataGenerator-request>"
  "face_tracking/DataGeneratorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DataGenerator-request)))
  "Returns string type for a service object of type 'DataGenerator-request"
  "face_tracking/DataGeneratorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DataGenerator-request>)))
  "Returns md5sum for a message object of type '<DataGenerator-request>"
  "d82dc6474dd88dad5e1615ab1b2ca74c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DataGenerator-request)))
  "Returns md5sum for a message object of type 'DataGenerator-request"
  "d82dc6474dd88dad5e1615ab1b2ca74c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DataGenerator-request>)))
  "Returns full string definition for message of type '<DataGenerator-request>"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DataGenerator-request)))
  "Returns full string definition for message of type 'DataGenerator-request"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DataGenerator-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DataGenerator-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DataGenerator-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude DataGenerator-response.msg.html

(cl:defclass <DataGenerator-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass DataGenerator-response (<DataGenerator-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DataGenerator-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DataGenerator-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name face_tracking-srv:<DataGenerator-response> is deprecated: use face_tracking-srv:DataGenerator-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <DataGenerator-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader face_tracking-srv:success-val is deprecated.  Use face_tracking-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <DataGenerator-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader face_tracking-srv:message-val is deprecated.  Use face_tracking-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DataGenerator-response>) ostream)
  "Serializes a message object of type '<DataGenerator-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DataGenerator-response>) istream)
  "Deserializes a message object of type '<DataGenerator-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DataGenerator-response>)))
  "Returns string type for a service object of type '<DataGenerator-response>"
  "face_tracking/DataGeneratorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DataGenerator-response)))
  "Returns string type for a service object of type 'DataGenerator-response"
  "face_tracking/DataGeneratorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DataGenerator-response>)))
  "Returns md5sum for a message object of type '<DataGenerator-response>"
  "d82dc6474dd88dad5e1615ab1b2ca74c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DataGenerator-response)))
  "Returns md5sum for a message object of type 'DataGenerator-response"
  "d82dc6474dd88dad5e1615ab1b2ca74c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DataGenerator-response>)))
  "Returns full string definition for message of type '<DataGenerator-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DataGenerator-response)))
  "Returns full string definition for message of type 'DataGenerator-response"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DataGenerator-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DataGenerator-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DataGenerator-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DataGenerator)))
  'DataGenerator-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DataGenerator)))
  'DataGenerator-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DataGenerator)))
  "Returns string type for a service object of type '<DataGenerator>"
  "face_tracking/DataGenerator")