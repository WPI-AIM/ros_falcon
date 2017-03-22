; Auto-generated. Do not edit!


(in-package rosfalcon-msg)


;//! \htmlinclude falconForces.msg.html

(defclass <falconForces> (ros-message)
  ((X
    :reader X-val
    :initarg :X
    :type fixnum
    :initform 0)
   (Y
    :reader Y-val
    :initarg :Y
    :type fixnum
    :initform 0)
   (Z
    :reader Z-val
    :initarg :Z
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <falconForces>) ostream)
  "Serializes a message object of type '<falconForces>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'X)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'X)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'Y)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'Y)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'Z)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'Z)) ostream)
)
(defmethod deserialize ((msg <falconForces>) istream)
  "Deserializes a message object of type '<falconForces>"
  (setf (ldb (byte 8 0) (slot-value msg 'X)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'X)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'Y)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'Y)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'Z)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'Z)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<falconForces>)))
  "Returns string type for a message object of type '<falconForces>"
  "rosfalcon/falconForces")
(defmethod md5sum ((type (eql '<falconForces>)))
  "Returns md5sum for a message object of type '<falconForces>"
  "b4669138e2c3f68ffec8f31ed47c17a1")
(defmethod message-definition ((type (eql '<falconForces>)))
  "Returns full string definition for message of type '<falconForces>"
  (format nil "int16 X~%int16 Y~%int16 Z~%~%~%"))
(defmethod serialization-length ((msg <falconForces>))
  (+ 0
     2
     2
     2
))
(defmethod ros-message-to-list ((msg <falconForces>))
  "Converts a ROS message object to a list"
  (list '<falconForces>
    (cons ':X (X-val msg))
    (cons ':Y (Y-val msg))
    (cons ':Z (Z-val msg))
))
