import numpy as np
import cv2 
from matplotlib import pyplot as plt

img = cv2.imread('/Users/sara/Pictures/10-10-6k.jpg')
assert img is not None, "file could not be read, check with os.path.exists()"

kernel = np.ones((25,25),np.float32)/(25**2)
dst = cv2.filter2D(img,-1,kernel)

cv2.imwrite('convolved image.png', dst)