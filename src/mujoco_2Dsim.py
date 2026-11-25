import mujoco
import numpy as np
from Indy7_2D3DOF import INDY7_2D_JOINTS as J
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parent.parent))
TRAJ_DIR = Path(__file__).resolve().parent.parent / "data" / "trajectories"
SIM_DIR = Path(__file__).resolve().parent.parent / "data" / "sim"
from config.indy7_dimensions import TCP_OFFSET
TCP_POS = np.array([0,0,TCP_OFFSET])

def build_model(
        #TCP의 페이로드 관련 정보 추가 입력
        pos: np.ndarray | None = np.array([0.0, 0.0, 0.0]), 
        mass : float | None = None,
        ipos : np.ndarray | None = np.array([0.02, 0.0, 0.04]),
        inertia : np.ndarray | None = np.array([0.015, 0.015, 0.015])
):
    spec = mujoco.MjSpec.from_file(str(Path(__file__).resolve().parent.parent/"model"/"indy7.urdf" ))
    spec.compiler.fusestatic = False
    if mass is not None:
        spec.body("tcp").add_body(name="payload", pos=pos, mass=mass, ipos= ipos, inertia=inertia)
        #ipos = [-cy, 0, cx]
    model = spec.compile()
    model.dof_damping[:] = 0
    model.dof_frictionloss[:] = 0
    model.opt.gravity[:] = [0.0, 0.0, -9.80665]
    return model

def simulate_dynamics(
    model: mujoco.MjModel,
    q: np.ndarray,
    qdot: np.ndarray,
    qddot: np.ndarray,
) -> np.ndarray:
    data = mujoco.MjData(model)
    idx = [1, 2, 4]
    N = len(q)
    torque = np.zeros((N, 3))
    for i in range(N):
        data.qpos[idx] = q[i]
        data.qvel[idx] = qdot[i]
        data.qacc[idx] = qddot[i]
        mujoco.mj_inverse(model, data)
        torque[i] = data.qfrc_inverse[idx]
    return torque

def comvert_form(pos, mass, ipos, inertia):
    offset = pos + ipos
    cx= 

if __name__ == "__main__":
    m_loaded = build_model(mass=2.5)
    m_free = build_model()
    print("payload mass:",m_loaded.body("payload").mass)
    print("diff of total mass:", m_loaded.body_mass.sum() - m_free.body_mass.sum())

    q = np.array([[0.0, 0.0, 0.0], [0.3, -0.5, 0.2]])
    z = np.zeros_like(q)
    print(simulate_dynamics(m_loaded, q, z, z) - simulate_dynamics(m_free, q, z, z))
    