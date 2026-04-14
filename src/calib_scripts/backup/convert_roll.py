import numpy as np
from tf.transformations import euler_from_matrix

T = np.array([
    [ 0.9597, -0.0435, 0.2781],
    [ 0.0576,  0.9666, -0.2514],
    [-0.0461,  0.2525, 0.9668]
])
rpy = euler_from_matrix(T)
print("RPY (rad):", rpy)
