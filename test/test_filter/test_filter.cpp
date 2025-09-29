#include <unity.h>
#include "filter.h"
#include <math.h>

// Define the tolerance for floating point comparisons
#define FLOAT_TOLERANCE 0.0001

// Test a simple FIR moving average filter
void test_fir_moving_average() {
    const int order = 4;
    const float b_coeffs[] = {0.2, 0.2, 0.2, 0.2, 0.2};
    const float a_coeffs[] = {1.0, 0.0, 0.0, 0.0, 0.0};
    const float expected_output[] = {0.2, 0.4, 0.6, 0.8, 1.0, 1.0, 1.0};

    IIRFilter filter;
    filter.init(order, b_coeffs, a_coeffs);

    for (int i = 0; i < 7; i++) {
        float output = filter.filter(1.0);
        TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, expected_output[i], output);
    }
}

// Test a simple IIR leaky integrator
void test_iir_leaky_integrator() {
    const int order = 1;
    const float b_coeffs[] = {1.0, 0.0}; // b0=1, b1=0
    const float a_coeffs[] = {1.0, -0.9}; // a0=1, a1=-0.9 => y[n] = x[n] + 0.9*y[n-1]

    IIRFilter filter;
    filter.init(order, b_coeffs, a_coeffs);

    float output = 0;
    // Step input x[n] = 1.0
    output = filter.filter(1.0); // y[0] = x[0] = 1.0
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 1.0, output);

    output = filter.filter(1.0); // y[1] = x[1] + 0.9*y[0] = 1.0 + 0.9*1.0 = 1.9
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 1.9, output);

    output = filter.filter(1.0); // y[2] = x[2] + 0.9*y[1] = 1.0 + 0.9*1.9 = 2.71
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 2.71, output);
}


int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_fir_moving_average);
    RUN_TEST(test_iir_leaky_integrator);
    UNITY_END();
    return 0;
}