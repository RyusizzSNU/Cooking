
import scipy.spatial.transform.rotation as r
from scipy.spatial.transform import Rotation as R

a = R.from_quat([0, 0, 0, 1])
print(type(a))
print(isinstance(a, R))