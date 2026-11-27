import mujoco
import numpy as np
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parent.parent))
TRAJ_DIR = Path(__file__).resolve().parent.parent / "data" / "trajectories"
RESULT_DIR = Path(__file__).resolve().parent.parent / "data" / "results"
from config.indy7_dimensions import TCP_OFFSET
TCP_POS = np.array([0,0,TCP_OFFSET])

def build_model(payload: dict | None = None):
    spec = mujoco.MjSpec.from_file(str(Path(__file__).resolve().parent.parent/"model"/"indy7.urdf" ))
    spec.compiler.fusestatic = False
    if payload is not None:
        spec.body("tcp").add_body(name="payload", **payload)
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
    contact_idx = []
    for i in range(N):
        data.qpos[idx] = q[i]
        data.qvel[idx] = qdot[i]
        data.qacc[idx] = qddot[i]
        mujoco.mj_inverse(model, data)
        if data.ncon > 0:
            contact_idx.append(i)
        torque[i] = data.qfrc_inverse[idx]
    if contact_idx:
        raise RuntimeError(f"접촉 발생: {len(contact_idx)}/{N} 샘플")
    return torque

def comvert_form(pos, mass, ipos, inertia):
    offset = pos + ipos
    cx = offset[2]
    cy = -offset[0]
    Io = inertia[1] + mass*( cx**2 + cy**2 )
    return mass, cx, cy, Io

def run_sim(traj_id: int, payload: dict | None = None):
    traj = np.load(TRAJ_DIR / f"optimized_trajectory_{traj_id:03d}.npz")
    model = build_model(payload)
    torque = simulate_dynamics(model, traj["q"], traj["qdot"], traj["qddot"])
    out_dir = RESULT_DIR / f"trajectory_{traj_id:03d}"
    out_dir.mkdir(parents=True, exist_ok=True)
    tag = "unloaded" if payload is None else "loaded"
    out = out_dir / f"sim_{tag}.npz"
    sim_data = dict(time=traj["time"], q=traj["q"], qdot=traj["qdot"], qddot=traj["qddot"], torque=torque)
    np.savez(out, **sim_data)
    print(f"저장: {out}  |torque|_max: {np.abs(torque).max(axis=0)}")
    return torque

if __name__ == "__main__":
    traj_id = int(sys.argv[1])
    TEST_PAYLOAD = dict(pos=np.array([0.0, 0.0, 0.0]), mass=2.5,
                        ipos=np.array([0.02, 0.0, 0.04]),
                        inertia=np.array([0.015, 0.015, 0.015]))
    run_sim(traj_id)
    run_sim(traj_id, TEST_PAYLOAD)
    