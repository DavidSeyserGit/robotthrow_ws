#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel, ApplyBodyWrench, GetLinkState
from geometry_msgs.msg import Pose, Wrench, Point
import math

# --- Constants ---
GRAVITY = 9.81  # Acceleration due to gravity (m/s^2)

def calculate_initial_velocity(distance, height, angle_degrees):
    """Calculates the initial velocity required for a parabolic throw.

    Args:
        distance (float): Horizontal distance to the target (m).
        height (float): Maximum height of the trajectory above the launch point (m).
        angle_degrees (float): Launch angle in degrees.

    Returns:
        float: Initial velocity (m/s).
    """
    angle_radians = math.radians(angle_degrees)
    v0 = math.sqrt((GRAVITY * distance**2) / (2 * math.cos(angle_radians)**2 * (distance * math.tan(angle_radians) + 2 * height)))
    return v0


def apply_force_for_velocity(initial_velocity, ball_mass, duration, angle_degrees):
    """Calculates and applies a force to achieve a desired initial velocity.

    Args:
        initial_velocity (float): Desired initial velocity (m/s).
        ball_mass (float): Mass of the ball (kg).
        duration (float): Duration over which to apply the force (seconds).
        angle_degrees (float): Angle of the force application in degrees.

    Returns:
        geometry_msgs.msg.Wrench: Wrench object representing the force.
    """
    force_magnitude = (ball_mass * initial_velocity) / duration
    angle_radians = math.radians(angle_degrees)
    force_x = force_magnitude * math.cos(angle_radians)
    force_z = force_magnitude * math.sin(angle_radians)

    wrench = Wrench()
    wrench.force.x = force_x
    wrench.force.y = 0
    wrench.force.z = force_z
    wrench.torque.x = 0
    wrench.torque.y = 0
    wrench.torque.z = 0

    return wrench

def spawn_and_throw():
    rospy.init_node('spawn_and_throw', log_level=rospy.DEBUG)
    rospy.loginfo("spawn_and_throw node started")

    # --- Gazebo Service Proxies ---
    rospy.loginfo("Waiting for gazebo/spawn_urdf_model service...")
    rospy.wait_for_service('gazebo/spawn_urdf_model')
    rospy.loginfo("gazebo/spawn_urdf_model service is available")

    rospy.loginfo("Waiting for gazebo/apply_body_wrench service...")
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    rospy.loginfo("/gazebo/apply_body_wrench service is available")

    try:
        spawn_model = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
        apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        rospy.loginfo("Service proxies created")

        # --- Ball Parameters ---
        ball_urdf = rospy.get_param("/ball_description")
        ball_mass = 0.1  # Mass of the ball in kg (adjust as needed)

        # --- Throw Parameters ---
        spawn_x = 0.0  # Initial X position
        spawn_y = 0.0  # Initial Y position
        spawn_z = 1.0  # Initial Z position (height above ground)
        target_distance = 5.0  # Horizontal distance to the target (meters)
        max_height = 2.0  # Maximum height of the trajectory (meters)
        launch_angle = 45.0  # Launch angle in degrees
        force_duration = 0.1 #Duration in seconds
        # --- Calculate Initial Velocity ---
        initial_velocity = calculate_initial_velocity(target_distance, max_height, launch_angle)
        rospy.loginfo(f"Calculated initial velocity: {initial_velocity:.2f} m/s")

        # --- Spawn the ball ---
        initial_pose = Pose()
        initial_pose.position.x = spawn_x
        initial_pose.position.y = spawn_y
        initial_pose.position.z = spawn_z
        initial_pose.orientation.x = 0
        initial_pose.orientation.y = 0
        initial_pose.orientation.z = 0
        initial_pose.orientation.w = 1
        rospy.loginfo("Spawning ball...")
        spawn_model(
            model_name="ball",
            model_xml=ball_urdf,
            robot_namespace="",
            initial_pose=initial_pose,
            reference_frame="world"
        )
        rospy.loginfo("Ball spawned successfully")

        # --- Apply the force ---
        wrench = apply_force_for_velocity(initial_velocity, ball_mass, force_duration, launch_angle)

        reference_point = Point()
        reference_point.x = 0
        reference_point.y = 0
        reference_point.z = 0
        rospy.loginfo("Applying force...")
        apply_body_wrench(
            body_name="ball::ball_link",
            reference_frame="ball::ball_link",
            wrench=wrench,
            start_time=rospy.Time.now(),
            duration=rospy.Duration(force_duration),
            reference_point=reference_point
        )
        rospy.loginfo("Force applied successfully")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    try:
        spawn_and_throw()
    except rospy.ROSInterruptException:
        pass
