; Auto-generated. Do not edit!


(in-package rosfalcon-msg)


;//! \htmlinclude gripperAngle.msg.html

(defclass <gripperAngle> (ros-message)
  ((angle
    :reader angle-val
    :initarg :angle
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <gripperAngle>) ostream)
  "Serializes a message object of type '<gripperAngle>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'angle)) ostream)
)
(defmethod deserialize ((msg <gripperAngle>) istream)
  "Deserializes a message object of type '<gripperAngle>"
  (setf (ldb (byte 8 0) (slot-value msg 'angle)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<gripperAngle>)))
  "Returns string type for a message object of type '<gripperAngle>"
  "rosfalcon/gripperAngle")
(defmethod md5sum ((type (eql '<gripperAngle>)))
  "Returns md5sum for a message object of type '<gripperAngle>"
  "fbb56a2fe56ac238b720fd3b47d0f11c")
(defmethod message-definition ((type (eql '<gripperAngle>)))
  "Returns full string definition for message of type '<gripperAngle>"
  (format nil "int8 angle~%~%~%"))
(defmethod serialization-length ((msg <gripperAngle>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <gripperAngle>))
  "Converts a ROS message object to a list"
  (list '<gripperAngle>
    (cons ':angle (angle-val msg))
))
