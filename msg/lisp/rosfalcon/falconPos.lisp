; Auto-generated. Do not edit!


(in-package rosfalcon-msg)


;//! \htmlinclude falconPos.msg.html

(defclass <falconPos> (ros-message)
  ((X
    :reader X-val
    :initarg :X
    :type float
    :initform 0.0)
   (Y
    :reader Y-val
    :initarg :Y
    :type float
    :initform 0.0)
   (Z
    :reader Z-val
    :initarg :Z
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <falconPos>) ostream)
  "Serializes a message object of type '<falconPos>"
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'X))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'Y))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'Z))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
)
(defmethod deserialize ((msg <falconPos>) istream)
  "Deserializes a message object of type '<falconPos>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'X) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'Y) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'Z) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<falconPos>)))
  "Returns string type for a message object of type '<falconPos>"
  "rosfalcon/falconPos")
(defmethod md5sum ((type (eql '<falconPos>)))
  "Returns md5sum for a message object of type '<falconPos>"
  "8219583d7802cc50be3e9ab911877ba5")
(defmethod message-definition ((type (eql '<falconPos>)))
  "Returns full string definition for message of type '<falconPos>"
  (format nil "float64 X~%float64 Y~%float64 Z~%~%~%"))
(defmethod serialization-length ((msg <falconPos>))
  (+ 0
     8
     8
     8
))
(defmethod ros-message-to-list ((msg <falconPos>))
  "Converts a ROS message object to a list"
  (list '<falconPos>
    (cons ':X (X-val msg))
    (cons ':Y (Y-val msg))
    (cons ':Z (Z-val msg))
))
