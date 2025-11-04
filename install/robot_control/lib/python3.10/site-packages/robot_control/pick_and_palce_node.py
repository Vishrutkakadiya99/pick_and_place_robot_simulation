import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import moveit_commander
import sys
import time

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        self.get_logger().info("Initializing MoveIt Commander...")

        # Initialize MoveIt Commander and the robot
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        # Define the planning group (must match your MoveIt config)
        self.ARM_GROUP_NAME = "arm_group" 
        self.arm_group = moveit_commander.MoveGroupCommander(self.ARM_GROUP_NAME)
        
        # Set planner parameters (optional)
        self.arm_group.set_planning_time(5.0)

        # Define pick and place coordinates (PLACEHOLDERS)
        self.PICK_TARGET = Pose()
        self.PICK_TARGET.position.x = 0.5
        self.PICK_TARGET.position.y = 0.0
        self.PICK_TARGET.position.z = 0.1 # On the table

        self.PLACE_TARGET = Pose()
        self.PLACE_TARGET.position.x = 0.5
        self.PLACE_TARGET.position.y = 0.5
        self.PLACE_TARGET.position.z = 0.1 

        # Give time for MoveIt and Gazebo to load
        time.sleep(3) 
        self.execute_task()

    def go_to_pose(self, pose: Pose, name: str):
        self.get_logger().info(f"Planning to {name}...")
        self.arm_group.set_pose_target(pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return success

    def execute_task(self):
        # 1. Move to a safe 'ready' position (Pre-Pick Approach)
        pre_pick_pose = self.PICK_TARGET
        pre_pick_pose.position.z += 0.2 # 20cm above object
        
        if not self.go_to_pose(pre_pick_pose, "Pre-Pick"):
            self.get_logger().error("Failed to reach Pre-Pick pose.")
            return

        # 2. Cartesian Path to Grasp
        waypoints = []
        wpose = self.arm_group.get_current_pose().pose
        
        # Descent (Z-axis only)
        wpose.position.z = self.PICK_TARGET.position.z + 0.01 # 1cm clearance
        waypoints.append(wpose)
        
        (plan, fraction) = self.arm_group.compute_cartesian_path(
                                waypoints, 0.01, 0.0)  # eef_step, jump_threshold

        if fraction > 0.9:
            self.get_logger().info("Successfully planned descent. Executing...")
            self.arm_group.execute(plan, wait=True)
        else:
            self.get_logger().error(f"Failed to plan descent. Only {fraction*100:.1f}% achieved.")
            return
        
        # 3. Simulate Grasp (In a real system, send a message to gripper controller)
        self.get_logger().info("Gripper closing (Simulated)...")
        # self.scene.attach_object(self.arm_group.get_end_effector_link(), "object_name")

        # 4. Move to safe 'retreat' position (Post-Pick Lift)
        post_pick_pose = self.arm_group.get_current_pose().pose
        post_pick_pose.position.z += 0.2
        if not self.go_to_pose(post_pick_pose, "Post-Pick Lift"): return

        # 5. Move to Pre-Place position
        pre_place_pose = self.PLACE_TARGET
        pre_place_pose.position.z += 0.2
        if not self.go_to_pose(pre_place_pose, "Pre-Place"): return

        # 6. Cartesian Path to Place
        waypoints = []
        wpose = self.arm_group.get_current_pose().pose
        wpose.position.z = self.PLACE_TARGET.position.z + 0.01
        waypoints.append(wpose)

        (plan, fraction) = self.arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)

        if fraction > 0.9:
            self.get_logger().info("Successfully planned placement descent. Executing...")
            self.arm_group.execute(plan, wait=True)
        else:
            self.get_logger().error("Failed to plan placement descent.")
            return

        # 7. Simulate Release
        self.get_logger().info("Gripper opening (Simulated)...")
        # self.scene.remove_attached_object(self.arm_group.get_end_effector_link(), "object_name")

        # 8. Return Home (Optional)
        # self.arm_group.set_named_target("home")
        # self.arm_group.go(wait=True)
        
        self.get_logger().info("Pick and Place task complete!")


def main(args=None):
    # MoveIt2 uses rclcpp for its C++ bindings, which requires rclpy initialization
    # and shutdown for cleanup.
    rclpy.init(args=args)
    
    # Use a threading executor for MoveIt Commander to work with ROS2 async nature
    executor = rclpy.executors.MultiThreadedExecutor()
    node = PickAndPlaceNode()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        moveit_commander.roscpp_shutdown() # Important cleanup

if __name__ == '__main__':
    main()
