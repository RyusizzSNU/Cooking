import numpy as np
from scipy.spatial.transform import Rotation as R
def to_44(mat_33, t = [0, 0, 0]):
    return np.concatenate(
                [np.concatenate(
                    [mat_33, [[0, 0, 0]]], 0
                ), [[t[0]], [t[1]], [t[2]], [1]]], 1
            )
def to_33(mat_44):
    return mat_44[:3, :3]

def tcp_to_affine(tcp):
    affine = R.from_rotvec(tcp[3:]).as_dcm()
    return to_44(affine, tcp[:3])

def dope_to_affine(dope, scale=1):
    return to_44(R.from_quat(dope[3:7]).as_dcm(), np.array(dope[:3]) * scale)