; Auto-generated. Do not edit!


(cl:in-package serial_manager-msg)


;//! \htmlinclude Param.msg.html

(cl:defclass <Param> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type cl:fixnum
    :initform 0)
   (param
    :reader param
    :initarg :param
    :type cl:float
    :initform 0.0))
)

(cl:defclass Param (<Param>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Param>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Param)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name serial_manager-msg:<Param> is deprecated: use serial_manager-msg:Param instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Param>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_manager-msg:header-val is deprecated.  Use serial_manager-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'param-val :lambda-list '(m))
(cl:defmethod param-val ((m <Param>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader serial_manager-msg:param-val is deprecated.  Use serial_manager-msg:param instead.")
  (param m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Param>) ostream)
  "Serializes a message object of type '<Param>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'header)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'param))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Param>) istream)
  "Deserializes a message object of type '<Param>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'header)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'param) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Param>)))
  "Returns string type for a message object of type '<Param>"
  "serial_manager/Param")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Param)))
  "Returns string type for a message object of type 'Param"
  "serial_manager/Param")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Param>)))
  "Returns md5sum for a message object of type '<Param>"
  "4ccd10a5f71453ae65c8aa51332c817e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Param)))
  "Returns md5sum for a message object of type 'Param"
  "4ccd10a5f71453ae65c8aa51332c817e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Param>)))
  "Returns full string definition for message of type '<Param>"
  (cl:format cl:nil "uint8 header~%float32 param~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Param)))
  "Returns full string definition for message of type 'Param"
  (cl:format cl:nil "uint8 header~%float32 param~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Param>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Param>))
  "Converts a ROS message object to a list"
  (cl:list 'Param
    (cl:cons ':header (header msg))
    (cl:cons ':param (param msg))
))
