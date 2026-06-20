// Fixed-point Qq format example (for MCUs without FPU)
#include <stdint.h>

#define Q 12
#define SCALE (1<<Q)

int32_t fx_mul(int32_t a, int32_t b){
    return (int32_t)(((int64_t)a * b) >> Q);
}

// Example 2x2 state update with fixed-point
void step_fx(int32_t x1, int32_t x2,
             int32_t* nx1, int32_t* nx2){
    // A = [[1, dt],[0,1]], B=[[0],[dt]]
    const int32_t dt = (int32_t)(0.01 * SCALE);
    const int32_t k1 = (int32_t)(2.0  * SCALE);
    const int32_t k2 = (int32_t)(0.5  * SCALE);

    int32_t u = -(fx_mul(k1, x1) + fx_mul(k2, x2));
    *nx1 = x1 + fx_mul(dt, x2);          // x1 + dt*x2
    *nx2 = x2 + fx_mul(dt, u);           // x2 + dt*u
}
