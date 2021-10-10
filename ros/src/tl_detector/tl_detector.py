#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from scipy.spatial import KDTree

STATE_COUNT_THRESHOLD = 3
LOOKAHEAD_WPS_TL = 100      # Number of waypoints ahead where to look for a traffic light

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
        
        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.camera_image = None
        self.lights = []

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoints', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoints_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        # rospy.loginfo("====== Image cb - image received ======")
        self.has_image = True
        self.camera_image = msg

        light_wp, state = self.process_traffic_lights()
        # rospy.loginfo("light_wp :{0}".format(light_wp))
        # rospy.loginfo("state :{0}".format(state))


        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        # rospy.loginfo("State count :{0}".format(self.state_count))
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            # rospy.loginfo("Publishing light_wp :{0}".format(light_wp))
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            # rospy.loginfo("Publishing last_wp :{0}".format(self.last_wp))
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
        Args:
            x, y: position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # Uses waypoint_tree defined in waypoint_cb
        closest_idx = self.waypoints_tree.query([x, y], 1)[1]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        imagID = self.light_classifier.get_classification(cv_image)
        rospy.loginfo("Image ID :{0}".format(imagID))

        # Get classification
        return imagID

#         For testing, comment all of the above and return the light state from the sim
#         return light.state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # rospy.loginfo("Processing traffic lights")
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        # (Coming from node config)
        stop_line_positions = self.config['stop_line_positions']

        if(self.pose):
            # Get closest waypoints
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
            # rospy.loginfo("Car_wp_idx: {0}".format(car_wp_idx))

            # Find closest traffic lights (if one exists)
            diff = LOOKAHEAD_WPS_TL
            for i, light in enumerate (self.lights):
                # Get stop line wp index
                line = stop_line_positions[i]
                tmp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                # Find closest wp
                d = tmp_wp_idx - car_wp_idx
                if ((d >= 0) and (d < diff)):
                    # rospy.loginfo("Length waypoynts: {0}".format(diff))
                    # rospy.loginfo("Distance stop line: {0}".format(d))
                    diff = d
                    closest_light = light
                    line_wp_idx = tmp_wp_idx

        if closest_light:
            rospy.loginfo("Found close light")
            # rospy.loginfo("Line_wp_idx: {0}".format(line_wp_idx))
            state = self.get_light_state(closest_light)

            if state == TrafficLight.RED:
                rospy.loginfo("Closest light is RED")
            elif state == TrafficLight.GREEN:
                rospy.loginfo("Closest light is GREEN")
            elif state == TrafficLight.YELLOW:
                rospy.loginfo("Closest light is YELLOW")

            return line_wp_idx, state

        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
