#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <lgpio.h>
#include <atomic>
#include <cstdint>

// 4x quadrature decoder using lgpio alert callbacks (lgGpioClaimAlert +
// lgGpioSetAlertsFuncEx).  Multiple encoders sharing the same chip handle are
// supported via an internal static registry.
class QuadratureEncoder {
public:
    // handle  — open lgpio chip handle
    // pin_a/b — channel A and B GPIO numbers (BCM)
    // cpr     — counts per revolution AFTER 4x decode (= 4 * encoder_ppr)
    QuadratureEncoder(int handle, int pin_a, int pin_b, int cpr);
    ~QuadratureEncoder();

    int64_t count()     const;  // raw 4x-decoded tick count
    float   angle_deg() const;  // degrees (unbounded)
    float   angle_rad() const;  // radians (unbounded)
    void    reset();            // zero the count

private:
    // Signature required by lgGpioAlertsFuncEx_t — no handle param
    static void alert_cb(int num_alerts, lgGpioAlert_p alerts, void* userdata);
    void on_edge(int gpio, int level);

    int handle_;
    int pin_a_;
    int pin_b_;
    int cpr_;

    std::atomic<int64_t> count_{0};
    int prev_ab_{0};  // previous (A<<1|B) — only touched by lgpio callback thread
};

#endif // ENCODER_HPP
