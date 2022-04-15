"""
testing offboard positon control with a simple takeoff script
"""

import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import math
import numpy

from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler


class OffbPosCtl:
    
    def mavrosTopicStringRoot(self, uavID=0):
        mav_topic_string = '/mavros/'
        return mav_topic_string

    def getWayPoints(self):
        
        locations = []

        init_angle = 0
        orientation1 = quaternion_from_euler(0, 0, init_angle)


        ln =  numpy.linspace(0, 8*numpy.pi, 360)
        z_points =  19.5 + 0.5 * numpy.sin(ln) 
        for i in range(360):
            x = self.rock_position[0] + self.rock_radius * math.cos(math.radians(i))
            y = self.rock_position[1] + self.rock_radius * math.sin(math.radians(i))
            z = z_points[i]

            qu = quaternion_from_euler(0.0, 0.0, math.radians(i-180))
            orientation_x = qu[0]
            orientation_y = qu[1]
            orientation_z = qu[2]
            orientation_w = qu[3]

            locations.append([x,y,z, orientation_x, orientation_y, orientation_z, orientation_w])

        locations = numpy.array((locations))

        return locations



    def __init__(self):

        self. curr_drone_pose = PoseStamped()
        self.waypointIndex = 0
        self.distThreshold = 1
        self.sim_ctr = 1

        self.des_pose = PoseStamped()
        self.isReadyToFly = False


        self.rock_position = [60.208, -12.502, 20]
        self.rock_radius = 3

        rospy.init_node('offboard_test', anonymous=True)
        pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        drone_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                                                 callback=self.drone_pose_cb)
        rover_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                                                 callback=self.rover_pose_cb)
        state_sub = rospy.Subscriber('/mavros/state', State, callback=self.drone_state_cb)
        attach = rospy.Publisher('/attach', String, queue_size=10)
        NUM_UAV = 2
        mode_proxy = [None for i in range(NUM_UAV)]
        arm_proxy = [None for i in range(NUM_UAV)]

        # Comm for drones
        for uavID in range(0, NUM_UAV):
            mode_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/set_mode', SetMode)
            arm_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/cmd/arming', CommandBool)

        rate = rospy.Rate(10)  # Hz
        rate.sleep()
        self.des_pose = self.copy_pose(self.curr_drone_pose)

        self.locations = self.getWayPoints()

        shape = self.locations.shape

        while not rospy.is_shutdown():
            print self.sim_ctr, shape[0], self.waypointIndex
            success = [None for i in range(NUM_UAV)]
            for uavID in range(0, NUM_UAV):
                try:
                    success[uavID] = mode_proxy[uavID](1, 'OFFBOARD')
                except rospy.ServiceException, e:
                    print ("mavros/set_mode service call failed: %s" % e)

            success = [None for i in range(NUM_UAV)]
            for uavID in range(0, NUM_UAV):
                rospy.wait_for_service(self.mavrosTopicStringRoot(uavID) + '/cmd/arming')

            for uavID in range(0, NUM_UAV):
                try:
                    success[uavID] = arm_proxy[uavID](True)
                except rospy.ServiceException, e:
                    print ("mavros1/set_mode service call failed: %s" % e)

            if self.waypointIndex >= shape[0]:
                self.waypointIndex = 0
                self.sim_ctr += 1

            if self.isReadyToFly:
                des = self.set_desired_pose().position
                azimuth = math.atan2(self.curr_rover_pose.pose.position.y - self.curr_drone_pose.pose.position.y,
                                     self.curr_rover_pose.pose.position.x - self.curr_drone_pose.pose.position.x)
                az_quat = quaternion_from_euler(0, 0, azimuth)

                curr = self.curr_drone_pose.pose.position

                self.waypointIndex += 1

            pose_pub.publish(self.des_pose)
            rate.sleep()

    def set_desired_pose(self):
        self.des_pose.pose.position.x = self.locations[self.waypointIndex, 0]
        self.des_pose.pose.position.y = self.locations[self.waypointIndex, 1]
        self.des_pose.pose.position.z = self.locations[self.waypointIndex, 2]
        self.des_pose.pose.orientation.x = self.locations[self.waypointIndex, 3]
        self.des_pose.pose.orientation.y = self.locations[self.waypointIndex, 4]
        self.des_pose.pose.orientation.z = self.locations[self.waypointIndex, 5]
        self.des_pose.pose.orientation.w = self.locations[self.waypointIndex, 6]
        if self.locations[self.waypointIndex, :].sum() == 0:
            self.des_pose.pose.position.x = self.curr_rover_pose.pose.position.x
            self.des_pose.pose.position.y = self.curr_rover_pose.pose.position.y
            self.des_pose.pose.position.z = max(self.curr_rover_pose.pose.position.z, 10)
            orientation = quaternion_from_euler(0, 0, 3.14/2)
            self.des_pose.pose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
        return self.des_pose.pose

    def copy_pose(self, pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        return copied_pose

    def drone_pose_cb(self, msg):
        self.curr_drone_pose = msg

    def rover_pose_cb(self, msg):
        self.curr_rover_pose = msg

    def drone_state_cb(self, msg):
        print msg.mode
        if (msg.mode == 'OFFBOARD'):
            self.isReadyToFly = True
            print "readyToFly"


if __name__ == "__main__":
    OffbPosCtl()
