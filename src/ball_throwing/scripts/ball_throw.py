#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel, ApplyBodyWrench, GetLinkState
from geometry_msgs.msg import Pose, Wrench, Point, Vector3
import math
import time # Import time for a small delay

# --- Global variable to potentially store ball mass if needed later ---
ball_mass = 0.1 # Default ball mass in kg (adjust as needed)

def spawn_model_func():
    """Spawns a model in Gazebo using the /ball_description rosparam."""

    rospy.init_node('spawn_and_throw_simple', log_level=rospy.DEBUG)
    rospy.loginfo("spawn_and_throw_simple node started")

    # --- Gazebo Service Proxies ---
    rospy.loginfo("Waiting for gazebo/spawn_urdf_model service...")
    rospy.wait_for_service('gazebo/spawn_urdf_model')
    rospy.loginfo("gazebo/spawn_urdf_model service is available")

    rospy.loginfo("Waiting for gazebo/apply_body_wrench service...")
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    rospy.loginfo("/gazebo/apply_body_wrench service is available")

    # We will also need GetLinkState to potentially get the ball's mass dynamically
    rospy.loginfo("Waiting for gazebo/get_link_state service...")
    rospy.wait_for_service('/gazebo/get_link_state')
    rospy.loginfo("/gazebo/get_link_state service is available")


    try:
        spawn_model = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
        # Note: apply_body_wrench and get_link_state proxies are created later

        # --- Model Parameters ---
        if not rospy.has_param("/ball_description"):
            rospy.logerr("ROS parameter '/ball_description' is not set.")
            rospy.logerr("Please ensure your launch file sets this parameter (e.g., using xacro).")
            return None # Return None to indicate spawn failed

        model_urdf = rospy.get_param("/ball_description")

        # --- Spawn Position ---
        spawn_x = 0.0  # Initial X position
        spawn_y = 0.0  # Initial Y position
        spawn_z = 1.0  # Initial Z position (height above ground)

        # --- Prepare the Pose message ---
        initial_pose = Pose()
        initial_pose.position.x = spawn_x
        initial_pose.position.y = spawn_y
        initial_pose.position.z = spawn_z
        initial_pose.orientation.x = 0
        initial_pose.orientation.y = 0
        initial_pose.orientation.z = 0
        initial_pose.orientation.w = 1  # Identity quaternion (no rotation)

        # --- Spawn the model ---
        model_name = "throwable_ball" # Use a descriptive name
        rospy.loginfo(f"Spawning model '{model_name}'...")

        resp = spawn_model(
            model_name=model_name,
            model_xml=model_urdf,
            robot_namespace="",
            initial_pose=initial_pose,
            reference_frame="world" # Spawn relative to the world frame
        )

        if resp.success:
            rospy.loginfo(f"Model '{model_name}' spawned successfully!")
            # Attempt to get the ball's mass after spawning
            try:
                get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
                # Assuming the link name is 'ball_link'. Adjust if needed.
                link_name = f"{model_name}::ball_link"
                state_resp = get_link_state(link_name=link_name, reference_frame="")
                if state_resp.success:
                    # Unfortunately, GetLinkState does not directly provide mass.
                    # We'll rely on the default ball_mass for now, but this is where
                    # you might dynamically get other properties if the service supported it.
                    rospy.loginfo(f"Successfully retrieved state for link '{link_name}'")
                else:
                     rospy.logwarn(f"Failed to get state for link '{link_name}': {state_resp.status_message}")
            except rospy.ServiceException as e:
                rospy.logwarn(f"Service call failed to get link state: {e}")


            return model_name # Return the spawned model name
        else:
            rospy.logerr(f"Failed to spawn model '{model_name}': {resp.status_message}")
            return None # Return None to indicate spawn failed

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return None
    except rospy.ROSException as e:
         rospy.logerr("ROS Exception: %s" % e)
         return None

def push_model(model_name):
    """Calculates and applies a force to achieve a desired initial velocity."""

    rospy.loginfo(f"Attempting to push model '{model_name}' with initial velocity")

    # --- Gazebo Service Proxy ---
    try:
        apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        rospy.loginfo("apply_body_wrench service proxy created for pushing")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to create apply_body_wrench service proxy: %s" % e)
        return

    # --- Desired Initial Velocity Parameters ---
    # Define the magnitude and angle of the desired initial velocity
    desired_velocity_magnitude = 3.0  # Desired speed in m/s - Adjust this!
    desired_velocity_angle_degrees = 45.0  # Angle relative to horizontal - Adjust this!

    # --- Parameters for applying the force ---
    # Apply the force over a slightly longer duration to be more stable
    force_application_duration = 0.01 # Duration over which the force is applied (seconds) - Adjust this!

    # --- Calculate Force Needed ---
    # We use the impulse-momentum theorem: Force * Duration = Mass * Change in Velocity
    # Assuming the ball starts at rest, Change in Velocity is just the Desired Velocity

    # Get ball mass (using global variable for now, as GetLinkState doesn't give it)
    current_ball_mass = ball_mass # Using the default or previously set mass

    # Calculate the desired velocity components
    velocity_x = desired_velocity_magnitude * math.cos(math.radians(desired_velocity_angle_degrees))
    velocity_z = desired_velocity_magnitude * math.sin(math.radians(desired_velocity_angle_degrees))

    # Calculate the required force components
    # Force_x = Mass * Velocity_x / Duration
    # Force_z = Mass * Velocity_z / Duration
    if force_application_duration <= 0:
         rospy.logerr("Force application duration must be greater than zero.")
         return

    force_x = (current_ball_mass * velocity_x) / force_application_duration
    force_z = (current_ball_mass * velocity_z) / force_application_duration

    # --- Prepare the Wrench message ---
    wrench = Wrench()
    wrench.force.x = force_x
    wrench.force.y = 0.0  # Assuming throw in the XZ plane
    wrench.force.z = force_z
    wrench.torque.x = 0.0
    wrench.torque.y = 0.0
    wrench.torque.z = 0.0

    # --- Reference Point for the force ---
    reference_point = Point()
    reference_point.x = 0.0
    reference_point.y = 0.0
    reference_point.z = 0.0

    # --- Apply the force ---
    # The body_name needs to be in the format "model_name::link_name"
    body_name_to_push = f"{model_name}::ball_link" # IMPORTANT: Adjust ball_link if your URDF uses a different link name

    rospy.loginfo(f"Applying calculated force to achieve velocity to '{body_name_to_push}'...")
    rospy.loginfo(f"  Desired Initial Velocity Magnitude: {desired_velocity_magnitude:.2f} m/s")
    rospy.loginfo(f"  Desired Initial Velocity Angle: {desired_velocity_angle_degrees:.1f} degrees")
    rospy.loginfo(f"  Calculated Force Magnitude: {math.sqrt(force_x**2 + force_z**2):.2f} N")
    rospy.loginfo(f"  Force Duration: {force_application_duration:.2f} seconds")

    try:
        resp = apply_body_wrench(
            body_name=body_name_to_push,
            reference_frame=body_name_to_push, # Apply force relative to the link itself
            wrench=wrench,
            start_time=rospy.Time.now(),
            duration=rospy.Duration(force_application_duration),
            reference_point=reference_point
        )

        if resp.success:
            rospy.loginfo(f"Force applied successfully to '{body_name_to_push}'!")
        else:
            rospy.logerr(f"Failed to apply force to '{body_name_to_push}': {resp.status_message}")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed when applying wrench: %s" % e)
    except rospy.ROSException as e:
         rospy.logerr("ROS Exception when applying wrench: %s" % e)


if __name__ == '__main__':
    # Spawn the model
    spawned_model_name = spawn_model_func()

    # If spawning was successful, wait a moment and then push it
    if spawned_model_name:
        rospy.loginfo("Waiting briefly after spawning before applying force...")
        time.sleep(1.0) # Give Gazebo a moment to register the spawned model
        push_model(spawned_model_name)