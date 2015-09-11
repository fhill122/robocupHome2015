; Auto-generated. Do not edit!


(cl:in-package utilities-msg)


;//! \htmlinclude gripperTestRequest.msg.html

(cl:defclass <gripperTestRequest> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (cmd
    :reader cmd
    :initarg :cmd
    :type cl:fixnum
    :initform 0)
   (limb
    :reader limb
    :initarg :limb
    :type cl:fixnum
    :initform 0))
)

(cl:defclass gripperTestRequest (<gripperTestRequest>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gripperTestRequest>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gripperTestRequest)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name utilities-msg:<gripperTestRequest> is deprecated: use utilities-msg:gripperTestRequest instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <gripperTestRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader utilities-msg:header-val is deprecated.  Use utilities-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <gripperTestRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader utilities-msg:cmd-val is deprecated.  Use utilities-msg:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'limb-val :lambda-list '(m))
(cl:defmethod limb-val ((m <gripperTestRequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader utilities-msg:limb-val is deprecated.  Use utilities-msg:limb instead.")
  (limb m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<gripperTestRequest>)))
    "Constants for message type '<gripperTestRequest>"
  '((:CMD_OPEN . 0)
    (:CMD_CLOSE . 1)
    (:LEFT . 3)
    (:RIGHT . 4)
    (:BOTH . 5))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'gripperTestRequest)))
    "Constants for message type 'gripperTestRequest"
  '((:CMD_OPEN . 0)
    (:CMD_CLOSE . 1)
    (:LEFT . 3)
    (:RIGHT . 4)
    (:BOTH . 5))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gripperTestRequest>) ostream)
  "Serializes a message object of type '<gripperTestRequest>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'cmd)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'limb)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gripperTestRequest>) istream)
  "Deserializes a message object of type '<gripperTestRequest>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'limb) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gripperTestRequest>)))
  "Returns string type for a message object of type '<gripperTestRequest>"
  "utilities/gripperTestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gripperTestRequest)))
  "Returns string type for a message object of type 'gripperTestRequest"
  "utilities/gripperTestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gripperTestRequest>)))
  "Returns md5sum for a message object of type '<gripperTestRequest>"
  "5ced861508a63eeda00fb12930a63d88")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gripperTestRequest)))
  "Returns md5sum for a message object of type 'gripperTestRequest"
  "5ced861508a63eeda00fb12930a63d88")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gripperTestRequest>)))
  "Returns full string definition for message of type '<gripperTestRequest>"
  (cl:format cl:nil "Header header~%~%#Request types~%	int8 CMD_OPEN = 0~%	int8 CMD_CLOSE = 1~%	int8 LEFT = 3~%	int8 RIGHT = 4~%	int8 BOTH = 5~%	~%int8 cmd~%int8 limb~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gripperTestRequest)))
  "Returns full string definition for message of type 'gripperTestRequest"
  (cl:format cl:nil "Header header~%~%#Request types~%	int8 CMD_OPEN = 0~%	int8 CMD_CLOSE = 1~%	int8 LEFT = 3~%	int8 RIGHT = 4~%	int8 BOTH = 5~%	~%int8 cmd~%int8 limb~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gripperTestRequest>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gripperTestRequest>))
  "Converts a ROS message object to a list"
  (cl:list 'gripperTestRequest
    (cl:cons ':header (header msg))
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':limb (limb msg))
))
