import os
import sys
import matplotlib.pyplot as plt
import numpy as np

image = np.load(sys.argv[1])["array"]
print(image.shape)
# np.savez_compressed(f"{os.path.splitext(sys.argv[1])[0]}.npz", array=image)
# image = (image - image.min()) / image.ptp()
plt.imshow(image)
plt.show()
