import numpy as np
import matplotlib.pyplot as plt
import robot_constants as constants

r1 = constants.FEMUR
r2 = constants.TIBULA

def solveIK(t, backLeg=False):
    # # The shoulder is at the origin and the target position is defined as being in the 3rd or 4th quadrants
    tx = t[0] * -1
    ty = t[1]
    tz = t[2]

    dT = np.sqrt(tx**2 + ty**2 + tz**2)
    kneeHipPaw = np.arccos((r1**2 + dT**2 - r2**2)/(2 * r1 * dT))
    pawKneeHip = np.arccos((r1**2 + r2**2 - dT**2)/(2 * r1 * r2))
    # print('pawKneeHip', np.rad2deg(pawKneeHip))
    # print('kneeHipPaw', np.rad2deg(kneeHipPaw))


    hip = np.arccos(-tx/dT) - kneeHipPaw
    # print(np.arctan(-tx/(ty)))
    # print((np.arccos((-tx)/dT) - np.pi/2) * -1)

    knee = np.pi - pawKneeHip

    if backLeg:
        hip = (((np.arccos((-tx)/dT) - np.pi/2) * -1) + kneeHipPaw)
        knee = - np.pi + pawKneeHip
    else:
        hip = (((np.arccos((-tx)/dT) - np.pi/2) * -1) - kneeHipPaw)
        knee = np.pi - pawKneeHip
    
    phi = np.arccos(ty/dT)
    if(tz < 0):
        phi *= -1


    # print(np.rad2deg(knee))
    if knee > np.deg2rad(60):
        print('out of range')
    # print("phi: " + str(phi))
    return hip, knee, phi


def plotLeg(theta1, theta2, phi, target):
    """
    Plots the leg configuration based on joint angles in 3D, including the roll motor (phi).
    
    Parameters:
    - theta1: Hip joint angle (femur vs. vertical).
    - theta2: Knee joint angle (tibia vs. femur extension).
    - phi: Roll motor angle (affects z-coordinate).
    """
    
    # Joint positions in 3D space
    hip = np.array([0, 0, 0])  # Hip at the origin
    knee = hip + np.array([r1 * np.sin(theta1), -r1 * np.cos(theta1), 0])
    foot = knee + np.array([r2 * np.sin(theta1 + theta2), -r2 * np.cos(theta1 + theta2), 0])
    
    # Apply the roll motor rotation (phi)
    roll_matrix = np.array([
        [1, 0, 0],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi), np.cos(phi)]
    ])
    
    rotated_foot = np.dot(roll_matrix, foot)

    # Plot the leg in 3D
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot the femur and tibia
    ax.plot([hip[0], knee[0]], [hip[1], knee[1]], [hip[2], knee[2]], 'bo-', label="Femur")
    ax.plot([knee[0], foot[0]], [knee[1], foot[1]], [knee[2], foot[2]], 'ro-', label="Tibia")

    # Plot the rotated foot (end effector)
    ax.scatter(rotated_foot[0], rotated_foot[1], rotated_foot[2], color='green', label='End-Effector (Foot)')

    # Plot target position
    ax.scatter(target[0], target[1], target[2], color='yellow', label='Target')

    # Labels and title
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_zlabel('Z Position (m)')
    ax.set_title("Quadrupedal Robot Leg Visualization with Roll Motor (phi)")
    
    # Display angles
    ax.text(hip[0] + 0.02, hip[1] - 0.02, hip[2], f"θ1: {np.degrees(theta1):.1f}°", color='red')
    ax.text(knee[0] + 0.02, knee[1] - 0.02, knee[2], f"θ2: {np.degrees(theta2):.1f}°", color='blue')
    ax.text(rotated_foot[0] + 0.02, rotated_foot[1] - 0.02, rotated_foot[2], f"φ: {np.degrees(phi):.1f}°", color='green')

    ax.legend()
    plt.show()

def main():
    """
    Main function to take user input for target position and plot the leg configuration.
    """
    try:
        # Get user input for the target position
        x = float(input("Enter the x-coordinate of the end-effector target position (m): "))
        y = float(input("Enter the y-coordinate of the end-effector target position (m): "))
        z = float(input("Enter the z-coordinate of the end-effector target position (m): "))
        
        # Calculate joint angles and roll motor angle (phi)
        theta1, theta2, phi = solveIK([x, y, z])
        print(f"Hip joint angle (θ1): {np.degrees(theta1):.2f}°")
        print(f"Knee joint angle (θ2): {np.degrees(theta2):.2f}°")
        print(f"Roll motor angle (φ): {np.degrees(phi):.2f}°")
        
        # Plot the leg with the updated configuration
        plotLeg(theta1, theta2, phi, [x, y, z])
    except ValueError as e:
        print(f"Error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    main()