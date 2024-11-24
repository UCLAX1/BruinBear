import numpy as np
import matplotlib.pyplot as plt

r1 = 0.177
r2 = 0.177

def solveIK(tx, ty, backLeg=False):
    # The shoulder is at the origin and the target position is defined as being in the 3rd or 4th quadrants
    tx = tx
    ty = ty
    dT = np.sqrt(tx**2 + ty**2)
    th1 = np.arccos((r1**2 + r2**2 - dT**2)/(2 * r1 * r2))
    th2 = np.arccos((r1**2 + dT**2 - r2**2)/(2 * r1 * dT))
    if tx == 0:
        th3 = np.pi/2 + th2
    else:
        th3 = np.arctan(ty/tx) + th2
    # if abs(th3) < 0.0001:
    #     th3 = np.pi

    if tx < 0:
        th3 += np.deg2rad(90)
    else:
        th3 -= np.deg2rad(90)
    # if backLeg:
    #     th1 += np.deg2rad(202)
    #     # th1 += (180 - th3)
    #     # th3 *= 1.2
    # # th1 = np.pi - th1
    if th3 <= (np.pi/2):
        if th3 > 0:
            th3 *= -1
    if backLeg:
        th3 *= -1
        th1 -= np.pi
        # th1 *= 2
        # th1 -= (th3 - (np.pi/2))
        print(th3)
        print(th1)
    else:
        th1 = np.pi - th1
    return th1, th3

def plotLeg(theta1, theta2):
    """
    Plots the leg configuration based on joint angles.
    
    Parameters:
    - theta1: Hip joint angle (femur vs. vertical).
    - theta2: Knee joint angle (tibia vs. femur extension).
    """
    
    # Joint positions
    hip = np.array([0, 0])  # Hip at the origin
    knee = hip + np.array([r1 * np.cos(theta1), r1 * np.sin(theta1)])
    foot = knee + np.array([r2 * np.cos(theta1 + theta2), r2 * np.sin(theta1 + theta2)])
    
    # Plot the leg
    plt.figure(figsize=(6, 6))
    plt.plot([hip[0], knee[0]], [hip[1], knee[1]], 'bo-', label="Femur")
    plt.plot([knee[0], foot[0]], [knee[1], foot[1]], 'ro-', label="Tibia")
    plt.scatter(*hip, color='black', label='Hip (Origin)')
    plt.scatter(*foot, color='green', label='End-Effector (Foot)')
    
    # Annotate angles
    plt.text(hip[0] + 0.02, hip[1] - 0.02, f"θ1: {np.degrees(theta1):.1f}°", color='red')
    plt.text(knee[0] + 0.02, knee[1] - 0.02, f"θ2: {np.degrees(theta2):.1f}°", color='blue')
    
    plt.axhline(0, color='gray', linestyle='--', linewidth=0.5)
    plt.axvline(0, color='gray', linestyle='--', linewidth=0.5)
    plt.title("Quadrupedal Robot Leg Visualization")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.axis("equal")
    plt.legend()
    plt.grid()
    plt.show()

def main():
    """
    Main function to take user input for target position and plot the leg configuration.
    """
    try:
        # Get user input for the target position
        x = float(input("Enter the x-coordinate of the end-effector target position (m): "))
        y = float(input("Enter the y-coordinate of the end-effector target position (m): "))
        
        # Calculate joint angles
        theta1, theta2 = solveIK(x, y)
        print(f"Hip joint angle (θ1): {np.degrees(theta1):.2f}°")
        print(f"Knee joint angle (θ2): {np.degrees(theta2):.2f}°")
        
        # Plot the leg
        plotLeg(theta1, theta2)
    except ValueError as e:
        print(f"Error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

# Run the main function
if __name__ == "__main__":
    main()