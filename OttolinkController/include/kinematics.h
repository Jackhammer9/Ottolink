#pragma once

#include "matrix.h"

class Kinematics
{
public:
    Kinematics() {}

    Matrix<3, 3> rodriguesRotation(const Matrix<1, 3>& k, float theta)
    {
        const float magnitude = k.magnitude();

        if (magnitude < 1e-3f) {
            return Matrix<3, 3>::identity();
        }

        Matrix<1, 3> T = k / magnitude;

        const float kx = T(0, 0);
        const float ky = T(0, 1);
        const float kz = T(0, 2);

        const float c = cosf(theta);
        const float s = sinf(theta);
        const float v = 1.0f - c;

        Matrix<3, 3> R = {
            c + kx*kx*v,     kx*ky*v - kz*s, kx*kz*v + ky*s,
            ky*kx*v + kz*s,  c + ky*ky*v,    ky*kz*v - kx*s,
            kz*kx*v - ky*s,  kz*ky*v + kx*s, c + kz*kz*v
        };

        return R;
    }

    Matrix<4, 4> homogeneousTransform(
        const Matrix<3, 3>& R,
        const Matrix<1, 3>& p)
    {
        Matrix<4, 4> T = Matrix<4, 4>::identity();

        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                T(row, col) = R(row, col);
            }
        }

        T(0, 3) = p(0, 0);
        T(1, 3) = p(0, 1);
        T(2, 3) = p(0, 2);

        return T;
    }

    Matrix<1, 3> forward(const Matrix<1, 4>& q)
    {
        Matrix<3, 3> I = Matrix<3, 3>::identity();
        Matrix<1, 3> zero = {0, 0, 0};

        Matrix<4, 4> T = homogeneousTransform(I, zero);

        Matrix<1, 3> k_pitch = {0, 1, 0};
        Matrix<1, 3> p_pitch = {0, 0, 0};
        Matrix<4, 4> T_pitch =
            homogeneousTransform(rodriguesRotation(k_pitch, q(0, 0)), p_pitch);

        Matrix<1, 3> k_yaw = {1, 0, 0.016f};
        Matrix<1, 3> p_yaw = {-0.0238f, 0.0633f, -0.0003f};
        Matrix<4, 4> T_yaw =
            homogeneousTransform(rodriguesRotation(k_yaw, q(0, 1)), p_yaw);

        Matrix<1, 3> k_roll = {0, 0.02f, 1};
        Matrix<1, 3> p_roll = {0.0235f, 0.0004f, -0.1010f};
        Matrix<4, 4> T_roll =
            homogeneousTransform(rodriguesRotation(k_roll, q(0, 2)), p_roll);

        Matrix<1, 3> k_elbow = {0.136f, 0.991f, 0.002f};
        Matrix<1, 3> p_elbow = {-0.0054f, 0.0302f, -0.1940f};
        Matrix<4, 4> T_elbow =
            homogeneousTransform(rodriguesRotation(k_elbow, q(0, 3)), p_elbow);

        Matrix<1, 3> p_ee = {0, 0, -0.18f};
        Matrix<4, 4> T_ee = homogeneousTransform(I, p_ee);

        T = T * T_pitch;
        T = T * T_yaw;
        T = T * T_roll;
        T = T * T_elbow;
        T = T * T_ee;

        Matrix<1, 3> position = {
            T(0, 3),
            T(1, 3),
            T(2, 3)
        };

        return position;
    }

    Matrix<3, 4> jacobian(const Matrix<1, 4>& q, float eps = 1e-3f)
    {
        Matrix<3, 4> J;

        for (int joint = 0; joint < 4; ++joint) {
            Matrix<1, 4> q_plus = q;
            Matrix<1, 4> q_minus = q;

            q_plus(0, joint) += eps;
            q_minus(0, joint) -= eps;

            Matrix<1, 3> p_plus = forward(q_plus);
            Matrix<1, 3> p_minus = forward(q_minus);
            Matrix<1, 3> dp = (p_plus - p_minus) / (2.0f * eps);

            J(0, joint) = dp(0, 0);
            J(1, joint) = dp(0, 1);
            J(2, joint) = dp(0, 2);
        }

        return J;
    }

    Matrix<4, 3> pseudoInverse(const Matrix<3, 4>& J, float damping)
    {
        Matrix<3, 3> JJT = J * J.transpose();
        Matrix<3, 3> I = Matrix<3, 3>::identity();
        Matrix<3, 3> A = JJT + I * (damping * damping);

        const float a = A(0, 0);
        const float b = A(0, 1);
        const float c = A(0, 2);
        const float d = A(1, 0);
        const float e = A(1, 1);
        const float f = A(1, 2);
        const float g = A(2, 0);
        const float h = A(2, 1);
        const float i = A(2, 2);

        const float det =
            a * (e*i - f*h)
          - b * (d*i - f*g)
          + c * (d*h - e*g);

        if (fabsf(det) < 1e-8f) {
            return Matrix<4, 3>::zeros();
        }

        Matrix<3, 3> A_inv = {
            (e*i - f*h) / det,
            (c*h - b*i) / det,
            (b*f - c*e) / det,

            (f*g - d*i) / det,
            (a*i - c*g) / det,
            (c*d - a*f) / det,

            (d*h - e*g) / det,
            (b*g - a*h) / det,
            (a*e - b*d) / det
        };

        return J.transpose() * A_inv;
    }

    Matrix<1, 4> inverseStep(
        const Matrix<1, 4>& q,
        const Matrix<1, 3>& target)
    {
        Matrix<1, 3> current = forward(q);
        Matrix<1, 3> error = target - current;

        Matrix<3, 4> J = jacobian(q);
        Matrix<4, 3> Jplus = pseudoInverse(J, 0.03f);

        Matrix<1, 4> dq = error * Jplus.transpose();

        dq = dq * 0.8f;

        // 0.10 radians is about 5.7 degrees
        const float max_dq = 0.10f;
        const float dq_norm = dq.magnitude();

        if (dq_norm > max_dq) {
            dq = dq * (max_dq / dq_norm);
        }

        return q + dq;
    }
};