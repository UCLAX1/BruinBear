import numpy as np
import matplotlib.pyplot as plt

r1 = 0.177
r2 = 0.177

tx = float(input('Enter x: '))
ty = float(input('Enter y: '))




dT = np.sqrt(tx**2 + ty**2)

th1 = np.arccos((r1**2 + r2**2 - dT**2)/(2 * r1 * r2))
th2 = np.arccos((r1**2 + dT**2 - r2**2)/(2 * r1 * dT))

if tx == 0:
    th3 = np.pi/2 + th2
    
else:
    
    if ty/tx > 0:
        th3 = np.arctan(ty/tx) + th2
    else: 
        th3 = np.pi - np.arctan(abs(ty/tx)) + th2
    if abs(th3) < 0.0001:
        th3 = np.pi
        # print("here")

# angle between femur and tibula CCW
print(th1)

# angle between the femur and the body CCW
print(th3)