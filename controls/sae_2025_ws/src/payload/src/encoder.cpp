#include "payload/encoder.hpp"

#include <cmath>

// Quadrature decode lookup table (4x).
// Index = (prev_ab << 2) | curr_ab  where ab = (A<<1)|B
// +1 = forward, -1 = reverse, 0 = no change or error
static const int8_t QEM[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0,
};

QuadratureEncoder::QuadratureEncoder(int handle, int pin_a, int pin_b, int cpr)
: handle_(handle), pin_a_(pin_a), pin_b_(pin_b), cpr_(cpr)
{
    // Claim both pins for edge alerts (supersedes any prior input claim)
    lgGpioClaimAlert(handle_, 0, LG_BOTH_EDGES, pin_a_, -1);
    lgGpioClaimAlert(handle_, 0, LG_BOTH_EDGES, pin_b_, -1);

    // Read initial state so the first edge is decoded correctly
    int a = lgGpioRead(handle_, pin_a_);
    int b = lgGpioRead(handle_, pin_b_);
    prev_ab_ = (a << 1) | b;

    // Register callback on each channel pin.
    lgGpioSetAlertsFunc(handle_, pin_a_, alert_cb, this);
    lgGpioSetAlertsFunc(handle_, pin_b_, alert_cb, this);
}

QuadratureEncoder::~QuadratureEncoder()
{
    lgGpioSetAlertsFunc(handle_, pin_a_, nullptr, nullptr);
    lgGpioSetAlertsFunc(handle_, pin_b_, nullptr, nullptr);
    lgGpioFree(handle_, pin_a_);
    lgGpioFree(handle_, pin_b_);
}

void QuadratureEncoder::alert_cb(int num_alerts, lgGpioAlert_p alerts, void* userdata)
{
    auto* self = static_cast<QuadratureEncoder*>(userdata);
    if (!self) {
        return;
    }

    for (int i = 0; i < num_alerts; ++i) {
        int gpio  = alerts[i].report.gpio;
        int level = alerts[i].report.level;
        if (gpio == self->pin_a_ || gpio == self->pin_b_) {
            self->on_edge(gpio, level);
        }
    }
}

void QuadratureEncoder::on_edge(int gpio, int level)
{
    int a = (gpio == pin_a_) ? level : lgGpioRead(handle_, pin_a_);
    int b = (gpio == pin_b_) ? level : lgGpioRead(handle_, pin_b_);
    int curr_ab = (a << 1) | b;

    count_.fetch_add(QEM[(prev_ab_ << 2) | curr_ab], std::memory_order_relaxed);
    prev_ab_ = curr_ab;
}

int64_t QuadratureEncoder::count() const
{
    return count_.load(std::memory_order_relaxed);
}

void QuadratureEncoder::reset()
{
    count_.store(0, std::memory_order_relaxed);
}

float QuadratureEncoder::angle_deg() const
{
    return static_cast<float>(count()) * 360.0f / static_cast<float>(cpr_);
}

float QuadratureEncoder::angle_rad() const
{
    return static_cast<float>(count()) * (2.0f * static_cast<float>(M_PI)) / static_cast<float>(cpr_);
}
