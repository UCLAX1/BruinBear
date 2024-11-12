import numpy as np
import matplotlib.pyplot as plt

r1 = 7
r2 = 7

tx = int(input('Enter x: '))
ty = int(input('Enter y: '))




dT = np.sqrt(tx**2 + ty**2);

th1 = np.arccos((r1**2 + r2**2 - dT**2)/(2 * r1 * r2))
th2 = np.arccos((r1**2 + dT**2 - r2**2)/(2 * r1 * dT))
th3 = np.arctan(ty/tx) + th2

# angle between femur and tibula CCW
print(th1 * 180/np.pi);

# angle between the femur and the body CCW
print(th3 * 180/np.pi);



