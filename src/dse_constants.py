"""
Constants
"""

EULER_ORDER = 'zyx'                 # The euler rotation order
EULER_ORDER_3D_OBS = 'z'            # The euler rotation order
INF_MATRIX_INITIAL = 100            # Initial information matrix (covariance) value. Equivalent to 0.01 standard deviation
INF_VECTOR_INITIAL = 0.01           # Initial information vector (states) value. x = Y^-1*y, equivalent to 1
MOTION_BASE_COVARIANCE = 0.000001   # Motion model covariance for velocity. Equivalent to 1 mm/sec standard deviation
GAZEBO_REFERENCE_OBJECT_NAME = 'aruco_marker::link'
GAZEBO_REFERENCE_OBJECT_ID = 0
ARUCO_TAG_TOTAL_SIZE = 512
ARUCO_TAG_NO_BORDER_SIZE = 340