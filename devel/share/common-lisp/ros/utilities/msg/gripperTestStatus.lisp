; Auto-generated. Do not edit!


(cl:in-package utilities-msg)


;//! \htmlinclude gripperTestStatus.msg.html

(cl:defclass <gripperTestStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (positionReached
    :reader positionReached
    :initarg :positionReached
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass gripperTestStatus (<gripperTestStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gripperTestStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gripperTestStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name utilities-msg:<gripperTestStatus> is deprecated: use utilities-msg:gripperTestStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <gripperTestStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader utilities-msg:header-val is deprecated.  Use utilities-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'positionReached-val :lambda-list '(m))
(cl:defmethod positionReached-val ((m <gripperTestStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader utilities-msg:positionReached-val is deprecated.  Use utilities-msg:positionReached instead.")
  (positionReached m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gripperTestStatus>) ostream)
  "Serializes a message object of type '<gripperTestStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'positionReached) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gripperTestStatus>) istream)
  "Deserializes a message object of type '<gripperTestStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'positionReached) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gripperTestStatus>)))
  "Returns string type for a message object of type '<gripperTestStatus>"
  "utilities/gripperTestStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gripperTestStatus)))
  "Returns string type for a message object of type 'gripperTestStatus"
  "utilities/gripperTestStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gripperTestStatus>)))
  "Returns md5sum for a message object of type '<gripperTestStatus>"
  "be758ce335e602c1d42b4663d6107ba5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gripperTestStatus)))
  "Returns md5sum for a message object of type 'gripperTestStatus"
  "be758ce335e602c1d42b4663d6107ba5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gripperTestStatus>)))
  "Returns full string definition for message of type '<gripperTestStatus>"
  (cl:format cl:nil "Header header~%bool positionReached #TODO: NYI~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gripperTestStatus)))
  "Returns full string definition for message of type 'gripperTestStatus"
  (cl:format cl:nil "Header header~%bool positionReached #TODO: NYI~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gripperTestStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gripperTestStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'gripperTestStatus
    (cl:cons ':header (header msg))
    (cl:cons ':positionReached (positionReached msg))
))
