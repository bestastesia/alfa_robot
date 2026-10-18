import ctypes
from pathlib import Path

import numpy as np

from v3_zero_model_candidate import OFFSETS_DEG,J2_LIMITS_DEG,JOINT_SIGNS


class RezeroAnalytic:
    def __init__(self):
        root=Path(__file__).resolve().parents[1]
        self.library=ctypes.CDLL(str(root/'generated/v3_analytic_071cb95/libv3_analytic_bridge.so'))
        array=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')
        self.library.v3_swivel.argtypes=[ctypes.c_int,array]
        self.library.v3_swivel.restype=ctypes.c_double
        self.library.v3_solve.argtypes=[ctypes.c_int,array,ctypes.c_double,array,array,ctypes.c_int]
        self.library.v3_solve.restype=ctypes.c_int
        self.signs=JOINT_SIGNS.copy()
        self.offsets=np.deg2rad(np.asarray([OFFSETS_DEG['left'],OFFSETS_DEG['right']]))
        self.limits=np.deg2rad([180.,90.,180.,145.,180.,110.,180.])
        self.lower=np.tile(-self.limits,(2,1));self.upper=np.tile(self.limits,(2,1))
        for arm,side in enumerate(('left','right')):
            self.lower[arm,1],self.upper[arm,1]=np.deg2rad(J2_LIMITS_DEG[side])

    def legacy(self,side,angles):
        values=self.offsets[side]+self.signs*np.asarray(angles)
        return np.ascontiguousarray((values+np.pi)%(2*np.pi)-np.pi,dtype=np.float64)

    def swivel(self,side,angles):
        return self.library.v3_swivel(side,self.legacy(side,angles))

    def solve(self,side,target,swivel,seed):
        output=np.zeros((16,7),dtype=np.float64)
        count=self.library.v3_solve(side,np.ascontiguousarray(target,dtype=np.float64),swivel,
            self.legacy(side,seed),output,16)
        converted=(output[:count]-self.offsets[side])*self.signs
        converted=(converted+np.pi)%(2*np.pi)-np.pi
        converted=converted[np.all((converted>=self.lower[side]-1e-9)&(converted<=self.upper[side]+1e-9),axis=1)]
        return converted[np.argsort(np.linalg.norm(converted-seed,axis=1))]
