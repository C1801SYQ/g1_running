/*
 * Additive G1 leg/IMU odometry for running missions.
 *
 * This module intentionally consumes only the generic RobotState already
 * exposed by rl_sar.  It does not change the upstream real-robot receiver or
 * FSM.  The six leg joints on each side are used to reconstruct the ankle
 * contact point in pelvis coordinates; the IMU quaternion rotates that point
 * into the world frame.  A support foot is selected from its height and the
 * consistency of its estimated world velocity.  When a foot is supporting,
 * the pelvis displacement is the negative change of that foot's world
 * relative position.
 *
 * This is deliberately conservative: if no support foot can be identified,
 * the estimator reports no sample instead of integrating commanded velocity.
 * That prevents a false 110 m completion on real hardware when joint/contact
 * data are unavailable.
 */
#ifndef G1_LEG_ODOMETRY_HPP
#define G1_LEG_ODOMETRY_HPP

#include "rl_sdk.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace g1_running_adapter
{

class G1LegOdometry
{
public:
    struct Config
    {
        // Foot contact is expected below the pelvis in the G1 URDF.  The
        // threshold is intentionally permissive for running crouch.
        float contact_z_max = -0.52f;
        // Relative-foot velocity after subtracting the previous body-speed
        // estimate.  Swing feet normally exceed this value.
        float contact_velocity_max = 2.0f;
        // Blend the selected support-foot velocity into the body velocity.
        float velocity_filter = 0.35f;
        // Ignore tiny contact updates that are dominated by encoder noise.
        float minimum_step_m = 0.0002f;
    };

    struct Estimate
    {
        bool initialized = false;
        bool valid = false;
        bool imu_valid = false;
        int contacts = 0;
        unsigned long samples = 0;
        float world_x = 0.0f;
        float world_y = 0.0f;
        float forward_m = 0.0f;
        float lateral_m = 0.0f;
        float forward_velocity_mps = 0.0f;
        float yaw_rad = 0.0f;
    };

    void Configure(const Config& config) { config_ = config; }

    static float YawFromQuaternion(const std::vector<float>& quaternion)
    {
        if (!QuaternionValid(quaternion))
            return 0.0f;
        const Mat3 rotation = RotationFromQuaternion(quaternion);
        return std::atan2(rotation.m[1][0], rotation.m[0][0]);
    }

    void Reset(const RobotState<float>& state, float start_yaw_rad)
    {
        initialized_ = false;
        have_previous_ = false;
        have_velocity_ = false;
        world_x_ = 0.0f;
        world_y_ = 0.0f;
        start_yaw_rad_ = start_yaw_rad;
        body_velocity_world_ = Vec3{};
        estimate_ = Estimate{};

        const Mat3 rotation = RotationFromQuaternion(state.imu.quaternion);
        std::array<Vec3, 2> feet{};
        if (!QuaternionValid(state.imu.quaternion) ||
            !FiniteMatrix(rotation) || !ReadFeet(state, feet))
            return;

        previous_rotation_ = rotation;
        previous_feet_ = feet;
        have_previous_ = true;
        initialized_ = true;
        estimate_.initialized = true;
        estimate_.imu_valid = true;
        estimate_.yaw_rad = std::atan2(rotation.m[1][0], rotation.m[0][0]);
    }

    Estimate Update(
        const RobotState<float>& state,
        float dt_seconds,
        float commanded_speed_mps,
        float heading_yaw_rad)
    {
        const float dt = std::clamp(dt_seconds, 1.0e-3f, 0.20f);
        const Mat3 rotation = RotationFromQuaternion(state.imu.quaternion);
        std::array<Vec3, 2> feet{};
        estimate_.contacts = 0;
        estimate_.valid = false;
        estimate_.imu_valid = QuaternionValid(state.imu.quaternion) &&
                              FiniteMatrix(rotation);
        if (!estimate_.imu_valid || !ReadFeet(state, feet))
        {
            estimate_.initialized = initialized_;
            return estimate_;
        }

        if (!initialized_ || !have_previous_)
        {
            Reset(state, heading_yaw_rad);
            return estimate_;
        }

        const float measured_yaw = std::atan2(rotation.m[1][0], rotation.m[0][0]);
        const Vec3 command_velocity{
            commanded_speed_mps * std::cos(heading_yaw_rad),
            commanded_speed_mps * std::sin(heading_yaw_rad),
            0.0f};
        // Keep the contact classifier usable during the first gait cycles,
        // while allowing measured leg odometry to replace the command hint.
        if (!have_velocity_)
        {
            body_velocity_world_ = command_velocity;
            have_velocity_ = true;
        }
        else
        {
            body_velocity_world_ =
                body_velocity_world_ * 0.90f + command_velocity * 0.10f;
        }

        std::array<Vec3, 2> relative_delta_world{};
        std::array<float, 2> contact_score{};
        std::array<bool, 2> contact_candidate{};
        for (std::size_t leg = 0; leg < 2; ++leg)
        {
            const Vec3 previous_world = Multiply(previous_rotation_, previous_feet_[leg]);
            const Vec3 current_world = Multiply(rotation, feet[leg]);
            relative_delta_world[leg] = current_world - previous_world;
            const Vec3 relative_velocity = relative_delta_world[leg] / dt;
            const Vec3 estimated_foot_velocity =
                relative_velocity + body_velocity_world_;
            const float horizontal_speed = std::hypot(
                estimated_foot_velocity.x, estimated_foot_velocity.y);
            const bool low_enough = feet[leg].z <= config_.contact_z_max;
            contact_candidate[leg] =
                low_enough && horizontal_speed <= config_.contact_velocity_max;
            contact_score[leg] = horizontal_speed +
                std::max(0.0f, feet[leg].z - config_.contact_z_max) * 4.0f;
        }

        // Use all feet that pass the conservative gate.  If neither passes,
        // retain the last pose and wait for the next support phase.
        Vec3 base_delta{};
        float weight_sum = 0.0f;
        for (std::size_t leg = 0; leg < 2; ++leg)
        {
            if (!contact_candidate[leg])
                continue;
            const float weight = 1.0f /
                std::max(0.05f, contact_score[leg] + 0.05f);
            base_delta += relative_delta_world[leg] * (-weight);
            weight_sum += weight;
            ++estimate_.contacts;
        }

        if (weight_sum > 0.0f)
        {
            base_delta = base_delta / weight_sum;
            const float horizontal_step = std::hypot(base_delta.x, base_delta.y);
            if (horizontal_step >= config_.minimum_step_m)
            {
                world_x_ += base_delta.x;
                world_y_ += base_delta.y;
                const Vec3 measured_velocity = base_delta / dt;
                body_velocity_world_ =
                    body_velocity_world_ * (1.0f - config_.velocity_filter) +
                    measured_velocity * config_.velocity_filter;
                ++estimate_.samples;
                estimate_.valid = true;
            }
        }

        previous_rotation_ = rotation;
        previous_feet_ = feet;
        estimate_.initialized = initialized_;
        estimate_.world_x = world_x_;
        estimate_.world_y = world_y_;
        estimate_.yaw_rad = measured_yaw;
        const float along = world_x_ * std::cos(start_yaw_rad_) +
                            world_y_ * std::sin(start_yaw_rad_);
        const float across = -world_x_ * std::sin(start_yaw_rad_) +
                             world_y_ * std::cos(start_yaw_rad_);
        estimate_.forward_m = along;
        estimate_.lateral_m = across;
        estimate_.forward_velocity_mps =
            body_velocity_world_.x * std::cos(start_yaw_rad_) +
            body_velocity_world_.y * std::sin(start_yaw_rad_);
        return estimate_;
    }

    const Estimate& LastEstimate() const { return estimate_; }

private:
    struct Vec3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;

        Vec3 operator+(const Vec3& other) const
        {
            return {x + other.x, y + other.y, z + other.z};
        }
        Vec3 operator-(const Vec3& other) const
        {
            return {x - other.x, y - other.y, z - other.z};
        }
        Vec3 operator*(float scalar) const
        {
            return {x * scalar, y * scalar, z * scalar};
        }
        Vec3 operator/(float scalar) const
        {
            return {x / scalar, y / scalar, z / scalar};
        }
        Vec3& operator+=(const Vec3& other)
        {
            x += other.x;
            y += other.y;
            z += other.z;
            return *this;
        }
    };

    struct Mat3
    {
        float m[3][3] = {{1.0f, 0.0f, 0.0f},
                         {0.0f, 1.0f, 0.0f},
                         {0.0f, 0.0f, 1.0f}};
    };

    static Mat3 Multiply(const Mat3& left, const Mat3& right)
    {
        Mat3 result{};
        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                result.m[row][col] = 0.0f;
                for (int k = 0; k < 3; ++k)
                    result.m[row][col] += left.m[row][k] * right.m[k][col];
            }
        }
        return result;
    }

    static Vec3 Multiply(const Mat3& matrix, const Vec3& vector)
    {
        return {
            matrix.m[0][0] * vector.x + matrix.m[0][1] * vector.y + matrix.m[0][2] * vector.z,
            matrix.m[1][0] * vector.x + matrix.m[1][1] * vector.y + matrix.m[1][2] * vector.z,
            matrix.m[2][0] * vector.x + matrix.m[2][1] * vector.y + matrix.m[2][2] * vector.z};
    }

    static Mat3 RotationX(float angle)
    {
        const float c = std::cos(angle);
        const float s = std::sin(angle);
        return {{{1.0f, 0.0f, 0.0f}, {0.0f, c, -s}, {0.0f, s, c}}};
    }

    static Mat3 RotationY(float angle)
    {
        const float c = std::cos(angle);
        const float s = std::sin(angle);
        return {{{c, 0.0f, s}, {0.0f, 1.0f, 0.0f}, {-s, 0.0f, c}}};
    }

    static Mat3 RotationZ(float angle)
    {
        const float c = std::cos(angle);
        const float s = std::sin(angle);
        return {{{c, -s, 0.0f}, {s, c, 0.0f}, {0.0f, 0.0f, 1.0f}}};
    }

    static Mat3 RotationFromQuaternion(const std::vector<float>& quaternion)
    {
        if (quaternion.size() < 4)
            return Mat3{};
        const float w = quaternion[0];
        const float x = quaternion[1];
        const float y = quaternion[2];
        const float z = quaternion[3];
        const float norm = std::sqrt(w * w + x * x + y * y + z * z);
        if (!std::isfinite(norm) || norm < 1.0e-6f)
            return Mat3{};
        const float nw = w / norm;
        const float nx = x / norm;
        const float ny = y / norm;
        const float nz = z / norm;
        return {{{1.0f - 2.0f * (ny * ny + nz * nz),
                  2.0f * (nx * ny - nz * nw),
                  2.0f * (nx * nz + ny * nw)},
                 {2.0f * (nx * ny + nz * nw),
                  1.0f - 2.0f * (nx * nx + nz * nz),
                  2.0f * (ny * nz - nx * nw)},
                 {2.0f * (nx * nz - ny * nw),
                  2.0f * (ny * nz + nx * nw),
                  1.0f - 2.0f * (nx * nx + ny * ny)}}};
    }

    static bool QuaternionValid(const std::vector<float>& quaternion)
    {
        if (quaternion.size() < 4)
            return false;
        const float norm = std::sqrt(
            quaternion[0] * quaternion[0] +
            quaternion[1] * quaternion[1] +
            quaternion[2] * quaternion[2] +
            quaternion[3] * quaternion[3]);
        return std::isfinite(norm) && norm >= 1.0e-6f;
    }

    static bool FiniteMatrix(const Mat3& matrix)
    {
        for (const auto& row : matrix.m)
            for (const float value : row)
                if (!std::isfinite(value))
                    return false;
        return true;
    }

    static void TranslateAndRotate(
        Mat3& rotation,
        Vec3& position,
        const Vec3& translation,
        const Mat3& fixed_rotation,
        const Mat3& joint_rotation)
    {
        position += Multiply(rotation, translation);
        rotation = Multiply(rotation, Multiply(fixed_rotation, joint_rotation));
    }

    static Vec3 FootPosition(const std::vector<float>& q, std::size_t offset)
    {
        const float side = offset == 0 ? 1.0f : -1.0f;
        Mat3 rotation{};
        Vec3 position{};
        auto apply = [&](const Vec3& translation,
                         const Mat3& fixed_rotation,
                         const Mat3& joint_rotation) {
            TranslateAndRotate(
                rotation, position, translation, fixed_rotation, joint_rotation);
        };

        // These transforms mirror robot_rl/robot_assets/g1/g1_29dof/
        // g1_29dof_official.urdf for the six leg joints.  The output point is
        // the center of the ankle contact rectangle, not the ankle joint.
        apply({0.0f, side * 0.064452f, -0.1027f}, Mat3{}, RotationY(q[offset + 0]));
        apply({0.0f, side * 0.052f, -0.030465f}, RotationY(-0.1749f), RotationX(q[offset + 1]));
        apply({0.025001f, 0.0f, -0.12412f}, Mat3{}, RotationZ(q[offset + 2]));
        apply({-0.078273f, side * 0.0021489f, -0.17734f}, RotationY(0.1749f), RotationY(q[offset + 3]));
        apply({0.0f, -side * 0.000094445f, -0.30001f}, Mat3{}, RotationY(q[offset + 4]));
        apply({0.0f, 0.0f, -0.017558f}, Mat3{}, RotationX(q[offset + 5]));
        return position + Multiply(rotation, Vec3{0.035f, 0.0f, -0.03f});
    }

    static bool ReadFeet(
        const RobotState<float>& state,
        std::array<Vec3, 2>& feet)
    {
        if (state.motor_state.q.size() < 12)
            return false;
        for (std::size_t leg = 0; leg < 2; ++leg)
        {
            feet[leg] = FootPosition(state.motor_state.q, leg * 6);
            if (!std::isfinite(feet[leg].x) ||
                !std::isfinite(feet[leg].y) ||
                !std::isfinite(feet[leg].z))
                return false;
        }
        return true;
    }

    Config config_{};
    bool initialized_ = false;
    bool have_previous_ = false;
    bool have_velocity_ = false;
    Mat3 previous_rotation_{};
    std::array<Vec3, 2> previous_feet_{};
    Vec3 body_velocity_world_{};
    float start_yaw_rad_ = 0.0f;
    float world_x_ = 0.0f;
    float world_y_ = 0.0f;
    Estimate estimate_{};
};

}  // namespace g1_running_adapter

#endif  // G1_LEG_ODOMETRY_HPP
