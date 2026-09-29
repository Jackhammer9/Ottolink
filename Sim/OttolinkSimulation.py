import mujoco
import mujoco.viewer
import pygame
import numpy as np

# MujoCo Initialization
model = mujoco.MjModel.from_xml_path("ottolink.xml")
data = mujoco.MjData(model)

site_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_SITE,
    "end_effector"
)

BASE_OFFSET = np.array([0.0, 0.0, 1.0])

# Rodrigues formula because the rotation axis are not streamlined (i should have modelled it better)
def rodrigues(k, theta):

    k = np.asarray(k, dtype=float)
    k = k / np.linalg.norm(k)

    kx, ky, kz = k

    c = np.cos(theta)
    s = np.sin(theta)
    v = 1.0 - c

    R = np.array([
        [
            c + kx*kx*v,
            kx*ky*v - kz*s,
            kx*kz*v + ky*s
        ],
        [
            ky*kx*v + kz*s,
            c + ky*ky*v,
            ky*kz*v - kx*s
        ],
        [
            kz*kx*v - ky*s,
            kz*ky*v + kx*s,
            c + kz*kz*v
        ]
    ])

    return R

# Homogenous coordinate transform
def transform(R, p):

    T = np.eye(4)

    T[:3, :3] = R
    T[:3, 3] = p

    return T

def fk(q):

    T = transform(
        np.eye(3),
        np.zeros(3)
    )

    R_pitch = rodrigues(
        [0, 1, 0],
        q[0]
    )

    T_pitch = transform(
        R_pitch,
        [0, 0, 0]
    )

    R_yaw = rodrigues(
        [1, 0, 0.016],
        q[1]
    )

    T_yaw = transform(
        R_yaw,
        [-0.0238, 0.0633, -0.0003]
    )


    R_roll = rodrigues(
        [0, 0.02, 1],
        q[2]
    )

    T_roll = transform(
        R_roll,
        [0.0235, 0.0004, -0.1010]
    )

    R_elbow = rodrigues(
        [0.136, 0.991, 0.002],
        q[3]
    )

    T_elbow = transform(
        R_elbow,
        [-0.0054, 0.0302, -0.1940]
    )

    T_ee = transform(
        np.eye(3),
        [0, 0, -0.18]
    )

    T = T @ T_pitch
    T = T @ T_yaw
    T = T @ T_roll
    T = T @ T_elbow
    T = T @ T_ee

    return T[:3, 3]


def numerical_jacobian(q, eps=1e-6):
    J = np.zeros((3, 4))
    for i in range(4):

        q_plus = q.copy()
        q_minus = q.copy()

        q_plus[i] += eps
        q_minus[i] -= eps

        p_plus = fk(q_plus)
        p_minus = fk(q_minus)

        J[:, i] = (
            p_plus - p_minus
        ) / (2 * eps)

    return J


def ik_step(q, target):
    current = fk(q)
    error = target - current
    J = numerical_jacobian(q)
    damping = 0.03

    dq = (
        J.T
        @ np.linalg.solve(
            J @ J.T + damping**2 * np.eye(3),
            error
        )
    )

    gain = 0.4
    dq *= gain
    max_dq = 0.02
    dq_norm = np.linalg.norm(dq)

    if dq_norm > max_dq:
        dq *= max_dq / dq_norm
    q_new = q + dq

    return q_new

q = np.array([
    0.0,     # Pitch
    0.0,     # Yaw
    0.0,     # Roll
    0.0      # Elbow
])

q = np.array([
    0.0,
    0.0,
    0.0,
    0.0
])

target = np.array([
    0.05,
    0.10,
    -0.30
])


# ---- FK ----
current = fk(q)

print("Current position:")
print(current)


# ---- IK STEP ----
q_new = ik_step(q, target)

print("\nNew q after IK step:")
print(q_new)


# ---- FK after IK ----
new_position = fk(q_new)

print("\nNew position:")
print(new_position)

# Pygame stuff below this point
pygame.init()

screen = pygame.display.set_mode(
    (500, 200)
)

pygame.display.set_caption(
    "Ottolink Cartesian Control"
)

clock = pygame.time.Clock()

STEP = 0.0005

with mujoco.viewer.launch_passive(
    model,
    data
) as viewer:

    running = True

    while running:
        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:

                if event.key == pygame.K_ESCAPE:
                    running = False

        keys = pygame.key.get_pressed()

        if keys[pygame.K_w]:
            target[0] += STEP

        if keys[pygame.K_s]:
            target[0] -= STEP

        if keys[pygame.K_a]:
            target[1] += STEP

        if keys[pygame.K_d]:
            target[1] -= STEP

        if keys[pygame.K_q]:
            target[2] += STEP

        if keys[pygame.K_e]:
            target[2] -= STEP

        q = ik_step(
            q,
            target
        )

        data.qpos[:4] = q

        mujoco.mj_forward(
            model,
            data
        )

        fk_position = fk(q)
        mujoco_world_position = data.site_xpos[site_id].copy()
        mujoco_local_position = (
            mujoco_world_position - BASE_OFFSET
        )
        error = target - fk_position
        error_norm = np.linalg.norm(error)

        print(
            f"\r"
            f"Target: {target} | "
            f"FK: {fk_position} | "
            f"MuJoCo: {mujoco_local_position} | "
            f"Error: {error_norm:.6f} m | "
            f"q: {q}",
            end=""
        )

        viewer.sync()

        clock.tick(100)

pygame.quit()
print("Controller stopped.")