; Auto-generated. Do not edit!


(cl:in-package vision-srv)


;//! \htmlinclude platePosition-request.msg.html

(cl:defclass <platePosition-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass platePosition-request (<platePosition-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <platePosition-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'platePosition-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-srv:<platePosition-request> is deprecated: use vision-srv:platePosition-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <platePosition-request>) ostream)
  "Serializes a message object of type '<platePosition-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <platePosition-request>) istream)
  "Deserializes a message object of type '<platePosition-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<platePosition-request>)))
  "Returns string type for a service object of type '<platePosition-request>"
  "vision/platePositionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'platePosition-request)))
  "Returns string type for a service object of type 'platePosition-request"
  "vision/platePositionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<platePosition-request>)))
  "Returns md5sum for a message object of type '<platePosition-request>"
  "5d86649d6e956fe5ee3b542339248e25")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'platePosition-request)))
  "Returns md5sum for a message object of type 'platePosition-request"
  "5d86649d6e956fe5ee3b542339248e25")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<platePosition-request>)))
  "Returns full string definition for message of type '<platePosition-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'platePosition-request)))
  "Returns full string definition for message of type 'platePosition-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <platePosition-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <platePosition-request>))
  "Converts a ROS message object to a list"
  (cl:list 'platePosition-request
))
;//! \htmlinclude platePosition-response.msg.html

(cl:defclass <platePosition-response> (roslisp-msg-protocol:ros-message)
  ((p1
    :reader p1
    :initarg :p1
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0))
   (p2
    :reader p2
    :initarg :p2
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0))
   (p3
    :reader p3
    :initarg :p3
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0))
   (p4
    :reader p4
    :initarg :p4
    :type (cl:vector cl:float)
   :initform (cl:make-array 2 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass platePosition-response (<platePosition-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <platePosition-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'platePosition-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vision-srv:<platePosition-response> is deprecated: use vision-srv:platePosition-response instead.")))

(cl:ensure-generic-function 'p1-val :lambda-list '(m))
(cl:defmethod p1-val ((m <platePosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-srv:p1-val is deprecated.  Use vision-srv:p1 instead.")
  (p1 m))

(cl:ensure-generic-function 'p2-val :lambda-list '(m))
(cl:defmethod p2-val ((m <platePosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-srv:p2-val is deprecated.  Use vision-srv:p2 instead.")
  (p2 m))

(cl:ensure-generic-function 'p3-val :lambda-list '(m))
(cl:defmethod p3-val ((m <platePosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-srv:p3-val is deprecated.  Use vision-srv:p3 instead.")
  (p3 m))

(cl:ensure-generic-function 'p4-val :lambda-list '(m))
(cl:defmethod p4-val ((m <platePosition-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vision-srv:p4-val is deprecated.  Use vision-srv:p4 instead.")
  (p4 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <platePosition-response>) ostream)
  "Serializes a message object of type '<platePosition-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'p1))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'p2))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'p3))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'p4))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <platePosition-response>) istream)
  "Deserializes a message object of type '<platePosition-response>"
  (cl:setf (cl:slot-value msg 'p1) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'p1)))
    (cl:dotimes (i 2)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'p2) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'p2)))
    (cl:dotimes (i 2)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'p3) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'p3)))
    (cl:dotimes (i 2)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'p4) (cl:make-array 2))
  (cl:let ((vals (cl:slot-value msg 'p4)))
    (cl:dotimes (i 2)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<platePosition-response>)))
  "Returns string type for a service object of type '<platePosition-response>"
  "vision/platePositionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'platePosition-response)))
  "Returns string type for a service object of type 'platePosition-response"
  "vision/platePositionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<platePosition-response>)))
  "Returns md5sum for a message object of type '<platePosition-response>"
  "5d86649d6e956fe5ee3b542339248e25")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'platePosition-response)))
  "Returns md5sum for a message object of type 'platePosition-response"
  "5d86649d6e956fe5ee3b542339248e25")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<platePosition-response>)))
  "Returns full string definition for message of type '<platePosition-response>"
  (cl:format cl:nil "float32[2] p1~%float32[2] p2~%float32[2] p3~%float32[2] p4~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'platePosition-response)))
  "Returns full string definition for message of type 'platePosition-response"
  (cl:format cl:nil "float32[2] p1~%float32[2] p2~%float32[2] p3~%float32[2] p4~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <platePosition-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'p1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'p2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'p3) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'p4) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <platePosition-response>))
  "Converts a ROS message object to a list"
  (cl:list 'platePosition-response
    (cl:cons ':p1 (p1 msg))
    (cl:cons ':p2 (p2 msg))
    (cl:cons ':p3 (p3 msg))
    (cl:cons ':p4 (p4 msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'platePosition)))
  'platePosition-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'platePosition)))
  'platePosition-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'platePosition)))
  "Returns string type for a service object of type '<platePosition>"
  "vision/platePosition")