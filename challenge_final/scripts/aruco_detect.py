#!/usr/bin/env python3

from fiducial_msgs.msg import FiducialTransformArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
import rospy
import tf2_ros
import tf_conversions


class ArucoDetect():
    def __init__(self):
        rospy.init_node("DetectR")

        rospy.Subscriber("fiducial_transforms", FiducialTransformArray, self.fiducial_cb)
        
        # Añadimos el publicador
        self.position_pub = rospy.Publisher("aruco_position", Pose, queue_size=10)

        self.aruco = FiducialTransformArray()
        self.needed_id = 721

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.quat = [0.0, 0.0, 0.0, 0.0]

        rospy.spin()

    def fiducial_cb(self, msg):
        self.aruco = msg
        if len(self.aruco.transforms) >= 1:
            found_id = self.aruco.transforms[0].fiducial_id

            #if found_id == self.needed_id:
            self.pos_x = round(self.aruco.transforms[0].transform.translation.x, 5)
            self.pos_y = round(self.aruco.transforms[0].transform.translation.y, 5)
            self.pos_z = round(self.aruco.transforms[0].transform.translation.z, 5)

            quat = [round(self.aruco.transforms[0].transform.rotation.x, 5),
                    round(self.aruco.transforms[0].transform.rotation.y, 5),
                    round(self.aruco.transforms[0].transform.rotation.z, 5),
                    round(self.aruco.transforms[0].transform.rotation.w, 5)]

            euler = euler_from_quaternion(quat)
            yaw = round(euler[2], 5)

            print("\nAruco ID:", str(found_id))
            print("\nPosition in x:", str(self.pos_x))
            print("\nPosition in y:", str(self.pos_y))
            print("\nPosition in z:", str(self.pos_z))
            print("\nOrientation in yaw:", str(yaw))

            # Publicar la posición como un Vector3
            pos_vector = Pose()
            pos_vector.position.x = self.pos_x
            pos_vector.position.y = self.pos_y
            pos_vector.position.z = self.pos_z
            pos_vector.orientation.z = yaw
            self.position_pub.publish(pos_vector)
            #else:
            #print("\nNeeded ID not found")


if __name__ == "__main__":
    rospy.init_node("DetectR")
    ArucoDetect()
