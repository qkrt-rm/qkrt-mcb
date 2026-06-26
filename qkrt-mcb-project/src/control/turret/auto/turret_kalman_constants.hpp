#ifndef TURRET_KALMAN_CONSTANTS_HPP_
#define TURRET_KALMAN_CONSTANTS_HPP_

namespace control::turret::kalman_config
{

static constexpr float VISION_DT = 1.0f / 60.0f;

static constexpr float KALMAN_A[36] = {
    1.0f, 0.0f, 0.0f, VISION_DT, 0.0f,      0.0f,
    0.0f, 1.0f, 0.0f, 0.0f,      VISION_DT, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f,      0.0f,      VISION_DT,
    0.0f, 0.0f, 0.0f, 1.0f,      0.0f,      0.0f,
    0.0f, 0.0f, 0.0f, 0.0f,      1.0f,      0.0f,
    0.0f, 0.0f, 0.0f, 0.0f,      0.0f,      1.0f
};

static constexpr float KALMAN_C[18] = {
    1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f
};

static constexpr float KALMAN_Q[36] = {
    0.1f, 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,
    0.0f, 0.1f,  0.0f,  0.0f,  0.0f,  0.0f,
    0.0f, 0.0f,  0.1f,  0.0f,  0.0f,  0.0f,
    0.0f, 0.0f,  0.0f,  1.0f, 0.0f,  0.0f,
    0.0f, 0.0f,  0.0f,  0.0f,  1.0f, 0.0f,
    0.0f, 0.0f,  0.0f,  0.0f,  0.0f,  1.0f
};

static constexpr float KALMAN_R[9] = {
    0.01f, 0.0f,  0.0f,
    0.0f,  0.01f, 0.0f,
    0.0f,  0.0f,  0.01f
};

static constexpr float KALMAN_P0[36] = {
    1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f
};

} // namespace control::turret::kalman_config

#endif // TURRET_KALMAN_CONSTANTS_HPP_