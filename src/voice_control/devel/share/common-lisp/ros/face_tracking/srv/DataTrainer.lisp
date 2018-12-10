; Auto-generated. Do not edit!


(cl:in-package face_tracking-srv)


;//! \htmlinclude DataTrainer-request.msg.html

(cl:defclass <DataTrainer-request> (roslisp-msg-protocol:ros-message)
  ((onoff
    :reader onoff
    :initarg :onoff
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DataTrainer-request (<DataTrainer-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DataTrainer-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DataTrainer-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name face_tracking-srv:<DataTrainer-request> is deprecated: use face_tracking-srv:DataTrainer-request instead.")))

(cl:ensure-generic-function 'onoff-val :lambda-list '(m))
(cl:defmethod onoff-val ((m <DataTrainer-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader face_tracking-srv:onoff-val is deprecated.  Use face_tracking-srv:onoff instead.")
  (onoff m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DataTrainer-request>) ostream)
  "Serializes a message object of type '<DataTrainer-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'onoff) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DataTrainer-request>) istream)
  "Deserializes a message object of type '<DataTrainer-request>"
    (cl:setf (cl:slot-value msg 'onoff) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DataTrainer-request>)))
  "Returns string type for a service object of type '<DataTrainer-request>"
  "face_tracking/DataTrainerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DataTrainer-request)))
  "Returns string type for a service object of type 'DataTrainer-request"
  "face_tracking/DataTrainerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DataTrainer-request>)))
  "Returns md5sum for a message object of type '<DataTrainer-request>"
  "07f48dc3148a93e6660a9d3c75b3c6af")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DataTrainer-request)))
  "Returns md5sum for a message object of type 'DataTrainer-request"
  "07f48dc3148a93e6660a9d3c75b3c6af")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DataTrainer-request>)))
  "Returns full string definition for message of type '<DataTrainer-request>"
  (cl:format cl:nil "bool onoff~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DataTrainer-request)))
  "Returns full string definition for message of type 'DataTrainer-request"
  (cl:format cl:nil "bool onoff~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DataTrainer-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DataTrainer-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DataTrainer-request
    (cl:cons ':onoff (onoff msg))
))
;//! \htmlinclude DataTrainer-response.msg.html

(cl:defclass <DataTrainer-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (num
    :reader num
    :initarg :num
    :type cl:integer
    :initform 0))
)

(cl:defclass DataTrainer-response (<DataTrainer-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DataTrainer-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DataTrainer-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name face_tracking-srv:<DataTrainer-response> is deprecated: use face_tracking-srv:DataTrainer-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <DataTrainer-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader face_tracking-srv:success-val is deprecated.  Use face_tracking-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'num-val :lambda-list '(m))
(cl:defmethod num-val ((m <DataTrainer-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader face_tracking-srv:num-val is deprecated.  Use face_tracking-srv:num instead.")
  (num m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DataTrainer-response>) ostream)
  "Serializes a message object of type '<DataTrainer-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DataTrainer-response>) istream)
  "Deserializes a message object of type '<DataTrainer-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DataTrainer-response>)))
  "Returns string type for a service object of type '<DataTrainer-response>"
  "face_tracking/DataTrainerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DataTrainer-response)))
  "Returns string type for a service object of type 'DataTrainer-response"
  "face_tracking/DataTrainerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DataTrainer-response>)))
  "Returns md5sum for a message object of type '<DataTrainer-response>"
  "07f48dc3148a93e6660a9d3c75b3c6af")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DataTrainer-response)))
  "Returns md5sum for a message object of type 'DataTrainer-response"
  "07f48dc3148a93e6660a9d3c75b3c6af")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DataTrainer-response>)))
  "Returns full string definition for message of type '<DataTrainer-response>"
  (cl:format cl:nil "bool success~%int64 num~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DataTrainer-response)))
  "Returns full string definition for message of type 'DataTrainer-response"
  (cl:format cl:nil "bool success~%int64 num~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DataTrainer-response>))
  (cl:+ 0
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DataTrainer-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DataTrainer-response
    (cl:cons ':success (success msg))
    (cl:cons ':num (num msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DataTrainer)))
  'DataTrainer-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DataTrainer)))
  'DataTrainer-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DataTrainer)))
  "Returns string type for a service object of type '<DataTrainer>"
  "face_tracking/DataTrainer")