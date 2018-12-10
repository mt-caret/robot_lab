; Auto-generated. Do not edit!


(cl:in-package face_tracking-msg)


;//! \htmlinclude Dist.msg.html

(cl:defclass <Dist> (roslisp-msg-protocol:ros-message)
  ((anglar
    :reader anglar
    :initarg :anglar
    :type cl:float
    :initform 0.0)
   (dist
    :reader dist
    :initarg :dist
    :type cl:integer
    :initform 0))
)

(cl:defclass Dist (<Dist>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Dist>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Dist)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name face_tracking-msg:<Dist> is deprecated: use face_tracking-msg:Dist instead.")))

(cl:ensure-generic-function 'anglar-val :lambda-list '(m))
(cl:defmethod anglar-val ((m <Dist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader face_tracking-msg:anglar-val is deprecated.  Use face_tracking-msg:anglar instead.")
  (anglar m))

(cl:ensure-generic-function 'dist-val :lambda-list '(m))
(cl:defmethod dist-val ((m <Dist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader face_tracking-msg:dist-val is deprecated.  Use face_tracking-msg:dist instead.")
  (dist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Dist>) ostream)
  "Serializes a message object of type '<Dist>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'anglar))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'dist)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Dist>) istream)
  "Deserializes a message object of type '<Dist>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'anglar) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dist) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Dist>)))
  "Returns string type for a message object of type '<Dist>"
  "face_tracking/Dist")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Dist)))
  "Returns string type for a message object of type 'Dist"
  "face_tracking/Dist")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Dist>)))
  "Returns md5sum for a message object of type '<Dist>"
  "2217c177b412cc5ac6a43674c38fcf6a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Dist)))
  "Returns md5sum for a message object of type 'Dist"
  "2217c177b412cc5ac6a43674c38fcf6a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Dist>)))
  "Returns full string definition for message of type '<Dist>"
  (cl:format cl:nil "float32 anglar~%int32 dist~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Dist)))
  "Returns full string definition for message of type 'Dist"
  (cl:format cl:nil "float32 anglar~%int32 dist~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Dist>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Dist>))
  "Converts a ROS message object to a list"
  (cl:list 'Dist
    (cl:cons ':anglar (anglar msg))
    (cl:cons ':dist (dist msg))
))
