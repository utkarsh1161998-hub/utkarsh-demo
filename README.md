import pybullet as p
import pybullet_data
import time
import math

def setup_simulation():
    """Initializes the PyBullet physics server and environment."""
    # Connect to GUI simulation
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Set environmental variables
    p.setGravity(0, 0, -9.81)
    p.resetDebugVisualizerCamera(cameraDistance=2.0, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 0.5])
    
    # Load environment assets
    p.loadURDF("plane.urdf")
    
    # Load a standard 6-Axis / 7-Axis robotic manipulator (KUKA LBR iiwa)
    robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
    return robot_id

def move_to_target(robot_id, target_pos, target_orn, end_effector_index=6):
    """
    Calculates Inverse Kinematics for a 6/7 axis robot and commands the joints.
    
    :param robot_id: The ID of the robot spawned in the simulation
    :param target_pos: List of [X, Y, Z] coordinates
    :param target_orn: Quaternion [X, Y, Z, W] defining tool orientation
    :param end_effector_index: The link index representing the robot's tool tip
    """
    # Calculate required joint angles using Inverse Kinematics (IK)
    joint_angles = p.calculateInverseKinematics(
        robot_id, 
        end_effector_index, 
        target_pos, 
        target_orn
    )
    
    # Get total number of joints
    num_joints = p.getNumJoints(robot_id)
    
    # Command active joints to move to the calculated positions
    # Note: KUKA has 7 joints; standard 6-axis operates on the same logic
    joint_idx = 0
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        if joint_info[2] != p.JOINT_FIXED:  # Only move active joints
            p.setJointMotorControl2(
                bodyIndex=robot_id,
                jointIndex=i,
                controlMode=p.CONTROL_MODE_POSITION,
                targetPosition=joint_angles[joint_idx],
                force=500
            )
            joint_idx += 1

def main():
    robot_id = setup_simulation()
    
    print("Simulation started. Moving robot in a square trajectory...")
    
    # Define Target Orientation (Tool facing downward)
    target_orientation = p.getQuaternionFromEuler([0, math.pi, 0])
    
    # Waypoints for the robot to follow [X, Y, Z]
    waypoints = [
        [0.4, 0.2, 0.5],
        [0.4, -0.2, 0.5],
        [0.6, -0.2, 0.5],
        [0.6, 0.2, 0.5]
    ]
    
    current_waypoint = 0
    step = 0
    
    try:
        while True:
            # Shift waypoints every 200 simulation steps (~2 seconds)
            if step % 200 == 0:
                target_position = waypoints[current_waypoint]
                print(f"Moving to Waypoint {current_waypoint + 1}: {target_position}")
                current_waypoint = (current_waypoint + 1) % len(waypoints)
            
            # Update joint positions
            move_to_target(robot_id, target_position, target_orientation)
            
            p.stepSimulation()
            time.sleep(1./240.) # Maintain 240Hz physics update
            step += 1
            
    except p.error:
        print("Simulation closed.")

if __name__ == "__main__":
    main()
