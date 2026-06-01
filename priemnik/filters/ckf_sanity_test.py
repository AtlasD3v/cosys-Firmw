import numpy as np
from filters import CKF as cub_calm_filter


def main():
    ckf = cub_calm_filter.CKF(dt=0.01)

    gyro = [0.0, 0.0, 0.0]
    acc_static = [0.0, 0.0, -9.81]   # неподвижный аппарат, Z-вверх

    for _ in range(100):
        ckf.predict_step(gyro, acc_static, dt=0.01)

    print("Static after 1s:")
    print("  pos =", ckf.state.pos)
    print("  vel =", ckf.state.vel)
    print("  q   =", ckf.state.q)

    gps = [5.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    pos_before = list(ckf.state.pos)
    ckf.correction_step(gps)

    print("\nAfter GPS correction:")
    print("  pos (before) =", pos_before)
    print("  pos =", ckf.state.pos)
    print("  vel =", ckf.state.vel)
    print("  q   =", ckf.state.q)

    P = ckf.P_matrix
    print("\nP checks:")
    print("  symmetric  =", np.allclose(P, P.T, atol=1e-8))
    print("  min eig(P) =", float(np.min(np.linalg.eigvalsh(P))))


# if __name__ == "__main__":
#     main()