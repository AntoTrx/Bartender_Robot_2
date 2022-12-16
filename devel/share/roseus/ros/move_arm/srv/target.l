;; Auto-generated. Do not edit!


(when (boundp 'move_arm::target)
  (if (not (find-package "MOVE_ARM"))
    (make-package "MOVE_ARM"))
  (shadow 'target (find-package "MOVE_ARM")))
(unless (find-package "MOVE_ARM::TARGET")
  (make-package "MOVE_ARM::TARGET"))
(unless (find-package "MOVE_ARM::TARGETREQUEST")
  (make-package "MOVE_ARM::TARGETREQUEST"))
(unless (find-package "MOVE_ARM::TARGETRESPONSE")
  (make-package "MOVE_ARM::TARGETRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass move_arm::targetRequest
  :super ros::object
  :slots (_data ))

(defmethod move_arm::targetRequest
  (:init
   (&key
    ((:data __data) nil)
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional (__data :null))
   (if (not (eq __data :null)) (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; bool _data
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _data
       (if _data (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _data
     (setq _data (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass move_arm::targetResponse
  :super ros::object
  :slots (_pose ))

(defmethod move_arm::targetResponse
  (:init
   (&key
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _pose __pose)
   self)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(defclass move_arm::target
  :super ros::object
  :slots ())

(setf (get move_arm::target :md5sum-) "66e12f7d13e74171b48748f4e38919a7")
(setf (get move_arm::target :datatype-) "move_arm/target")
(setf (get move_arm::target :request) move_arm::targetRequest)
(setf (get move_arm::target :response) move_arm::targetResponse)

(defmethod move_arm::targetRequest
  (:response () (instance move_arm::targetResponse :init)))

(setf (get move_arm::targetRequest :md5sum-) "66e12f7d13e74171b48748f4e38919a7")
(setf (get move_arm::targetRequest :datatype-) "move_arm/targetRequest")
(setf (get move_arm::targetRequest :definition-)
      "bool data
---
geometry_msgs/Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get move_arm::targetResponse :md5sum-) "66e12f7d13e74171b48748f4e38919a7")
(setf (get move_arm::targetResponse :datatype-) "move_arm/targetResponse")
(setf (get move_arm::targetResponse :definition-)
      "bool data
---
geometry_msgs/Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :move_arm/target "66e12f7d13e74171b48748f4e38919a7")


