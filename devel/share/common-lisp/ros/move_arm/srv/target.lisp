; Auto-generated. Do not edit!


(cl:in-package move_arm-srv)


;//! \htmlinclude target-request.msg.html

(cl:defclass <target-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass target-request (<target-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <target-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'target-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move_arm-srv:<target-request> is deprecated: use move_arm-srv:target-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <target-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_arm-srv:data-val is deprecated.  Use move_arm-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <target-request>) ostream)
  "Serializes a message object of type '<target-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'data) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <target-request>) istream)
  "Deserializes a message object of type '<target-request>"
    (cl:setf (cl:slot-value msg 'data) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<target-request>)))
  "Returns string type for a service object of type '<target-request>"
  "move_arm/targetRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'target-request)))
  "Returns string type for a service object of type 'target-request"
  "move_arm/targetRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<target-request>)))
  "Returns md5sum for a message object of type '<target-request>"
  "66e12f7d13e74171b48748f4e38919a7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'target-request)))
  "Returns md5sum for a message object of type 'target-request"
  "66e12f7d13e74171b48748f4e38919a7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<target-request>)))
  "Returns full string definition for message of type '<target-request>"
  (cl:format cl:nil "bool data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'target-request)))
  "Returns full string definition for message of type 'target-request"
  (cl:format cl:nil "bool data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <target-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <target-request>))
  "Converts a ROS message object to a list"
  (cl:list 'target-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude target-response.msg.html

(cl:defclass <target-response> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass target-response (<target-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <target-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'target-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move_arm-srv:<target-response> is deprecated: use move_arm-srv:target-response instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <target-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_arm-srv:pose-val is deprecated.  Use move_arm-srv:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <target-response>) ostream)
  "Serializes a message object of type '<target-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <target-response>) istream)
  "Deserializes a message object of type '<target-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<target-response>)))
  "Returns string type for a service object of type '<target-response>"
  "move_arm/targetResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'target-response)))
  "Returns string type for a service object of type 'target-response"
  "move_arm/targetResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<target-response>)))
  "Returns md5sum for a message object of type '<target-response>"
  "66e12f7d13e74171b48748f4e38919a7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'target-response)))
  "Returns md5sum for a message object of type 'target-response"
  "66e12f7d13e74171b48748f4e38919a7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<target-response>)))
  "Returns full string definition for message of type '<target-response>"
  (cl:format cl:nil "geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'target-response)))
  "Returns full string definition for message of type 'target-response"
  (cl:format cl:nil "geometry_msgs/Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <target-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <target-response>))
  "Converts a ROS message object to a list"
  (cl:list 'target-response
    (cl:cons ':pose (pose msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'target)))
  'target-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'target)))
  'target-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'target)))
  "Returns string type for a service object of type '<target>"
  "move_arm/target")