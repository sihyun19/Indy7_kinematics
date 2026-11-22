import mujoco
import numpy as np
from Indy7_2D3DOF import INDY7_2D_JOINTS as J
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parent.parent))
from config.indy7_dimensions import TCP_OFFSET

def build_model(
        pos: np.ndarray | None = np.array([0.0, 0.0, 0.06]), 
        mass : float | None = None,
        ipos : np.ndarray | None = np.array([0.02, 0.0, 0.04]),
        inertia : np.ndarray | None = np.array([0.015, 0.015, 0.015])
):
    spec = mujoco.MjSpec.from_file(str(Path(__file__).resolve().parent.parent/"model"/"indy7.urdf" ))
    if mass is not None:
        spec.body("link6").add_body(name="payload", pos=pos, mass=mass, ipos= ipos, inertia=inertia)
    
    model = spec.compile()
    model.dof_damping[:] = 0
    model.dof_frictionloss[:] = 0
    model.opt.gravity[:] = [0.0, 0.0, -9.80665]
    return model