#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class MissionManager:
  def __init__(self):
    rospy.init_node('mission_manager', anonymous=True)

    # Initialize state and pose variables
    self.current_state = State()
    self.current_pose = None

    # Define home position (deployment point)
    self.home_pose = PoseStamped()
    self.home_pose.header.frame_id = "map"
    self.home_pose.pose.position.x = 0.0
    self.home_pose.pose.position.y = 0.0
    self.home_pose.pose.position.z = 2.0  # takeoff altitude

    # Define drop waypoints (example with three drop locations)
    wp1 = PoseStamped()
    wp1.header.frame_id = "map"
    wp1.pose.position.x = 5.0
    wp1.pose.position.y = 0.0
    wp1.pose.position.z = 2.0

    wp2 = PoseStamped()
    wp2.header.frame_id = "map"
    wp2.pose.position.x = 5.0
    wp2.pose.position.y = 5.0
    wp2.pose.position.z = 2.0

    wp3 = PoseStamped()
    wp3.header.frame_id = "map"
    wp3.pose.position.x = 0.0
    wp3.pose.position.y = 5.0
    wp3.pose.position.z = 2.0

    self.waypoints = [wp1, wp2, wp3]

    # Set up publishers and subscribers
    self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    self.drop_pub = rospy.Publisher('/drop_package', String, queue_size=10)

    rospy.Subscriber('/mavros/state', State, self.state_cb)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)

    self.rate = rospy.Rate(20)  # 20 Hz

    # Wait for connection to flight controller (FCU)
    while not rospy.is_shutdown() and not self.current_state.connected:
        rospy.loginfo("Waiting for FCU connection...")
        self.rate.sleep()
    rospy.loginfo("Connected to FCU")

    # Wait for initial pose
    while not rospy.is_shutdown() and self.current_pose is None:
        rospy.loginfo("Waiting for current pose...")
        self.rate.sleep()

    # Prepare service proxies for arming and mode setting
    rospy.wait_for_service('/mavros/cmd/arming')
    rospy.wait_for_service('/mavros/set_mode')
    self.arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)

  def state_cb(self, msg):
    self.current_state = msg

  def pose_cb(self, msg):
    self.current_pose = msg

  def send_setpoint(self, target_pose):
    target_pose.header.stamp = rospy.Time.now()
    self.setpoint_pub.publish(target_pose)

  def distance_to_target(self, target_pose):
    if self.current_pose is None:
        return float('inf')
    dx = self.current_pose.pose.position.x - target_pose.pose.position.x
    dy = self.current_pose.pose.position.y - target_pose.pose.position.y
    dz = self.current_pose.pose.position.z - target_pose.pose.position.z
    return math.sqrt(dx*dx + dy*dy + dz*dz)

  def arm_and_set_mode(self):
    # Send a stream of setpoints (required before switching to OFFBOARD)
    for _ in range(100):
      self.send_setpoint(self.home_pose)
      self.rate.sleep()

    # Set OFFBOARD mode
    mode_resp = self.set_mode_srv(custom_mode="OFFBOARD")
    if mode_resp.mode_sent:
      rospy.loginfo("OFFBOARD mode enabled")
    else:
      rospy.logwarn("Failed to set OFFBOARD mode")

    # Arm the UAV
    arm_resp = self.arming_srv(True)
    if arm_resp.success:
      rospy.loginfo("UAV armed")
    else:
      rospy.logwarn("Failed to arm UAV")

  def fly_to(self, target_pose, tolerance=0.5, timeout=30):
    rospy.loginfo("Navigating to waypoint: x=%.2f, y=%.2f, z=%.2f" % (
      target_pose.pose.position.x,
      target_pose.pose.position.y,
      target_pose.pose.position.z))
    start_time = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
      self.send_setpoint(target_pose)
      dist = self.distance_to_target(target_pose)
      if dist < tolerance:
          rospy.loginfo("Reached waypoint")
          break
      if rospy.Time.now().to_sec() - start_time > timeout:
          rospy.logwarn("Timeout reached while navigating to waypoint")
          break
      self.rate.sleep()

  def drop_package(self, package_id):
    rospy.loginfo("Dropping package %s" % package_id)
    # Publish a drop command to simulate package drop
    drop_msg = String()
    drop_msg.data = "Drop package %s" % package_id
    self.drop_pub.publish(drop_msg)
    # Wait a bit to simulate drop duration
    rospy.sleep(2)

  def run_mission(self):
    self.arm_and_set_mode()

    rospy.loginfo("Taking off to home position")
    self.fly_to(self.home_pose)

    # Visit each drop waypoint and simulate package drop
    for idx, wp in enumerate(self.waypoints):
        self.fly_to(wp)
        self.drop_package(idx + 1)

    # After all drops, return to home
    rospy.loginfo("Mission complete. Returning to home.")
    self.fly_to(self.home_pose)
    rospy.loginfo("Returned to home. Landing procedure can be initiated.")

  def run(self):
    try:
      self.run_mission()
    except rospy.ROSInterruptException:
      pass

if __name__ == '__main__':
  manager = MissionManager()
  manager.run()
