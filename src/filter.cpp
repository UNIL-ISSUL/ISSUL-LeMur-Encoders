#include "filter.h"
#include <string.h>
#include <stdio.h>

/**
 * @brief Construct a new IIRFilter object.
 *
 * The constructor initializes the filter's order to 0 and clears all coefficient
 * and history buffers.
 */
IIRFilter::IIRFilter() {
    this->order = 0;
    // Initialize coefficients and history arrays to zero
    memset(b_coeffs, 0, sizeof(b_coeffs));
    memset(a_coeffs, 0, sizeof(a_coeffs));
    memset(x_history, 0, sizeof(x_history));
    memset(y_history, 0, sizeof(y_history));
}

/**
 * @brief Initializes the IIR filter with given coefficients.
 *
 * @param order The order of the filter. Must not exceed MAX_FILTER_ORDER.
 * @param b A pointer to an array of numerator coefficients.
 * @param a A pointer to an array of denominator coefficients.
 */
void IIRFilter::init(int order, const float* b, const float* a) {
    if (order > MAX_FILTER_ORDER) {
        // Handle error: filter order is too high for the defined maximum.
        // In a real application, you might want to log this error.
        printf("Error: Filter order %d exceeds maximum of %d\n", order, MAX_FILTER_ORDER);
        return;
    }

    this->order = order;

    // Copy coefficients into the private arrays.
    // Note: This assumes that the 'a' coefficients from python have a[0] = 1.
    memcpy(b_coeffs, b, (order + 1) * sizeof(float));
    memcpy(a_coeffs, a, (order + 1) * sizeof(float));

    // Reset history buffers to zero to ensure a clean start.
    memset(x_history, 0, sizeof(x_history));
    memset(y_history, 0, sizeof(y_history));
}

/**
 * @brief Applies the filter to a new input value.
 *
 * This function takes a new input value, updates the filter's history,
 * and computes the new output value based on the IIR filter equation.
 *
 * @param inputValue The new value to be filtered.
 * @return The filtered output value.
 */
float IIRFilter::filter(float inputValue) {
    // Shift the history of input and output values to make room for the new ones.
    for (int i = order; i > 0; i--) {
        x_history[i] = x_history[i - 1];
        y_history[i] = y_history[i - 1];
    }
    x_history[0] = inputValue;

    // Calculate the new output value using the difference equation.
    // y[n] = b[0]*x[n] + b[1]*x[n-1] + ... - a[1]*y[n-1] - a[2]*y[n-2] - ...
    float output = 0.0;
    for (int i = 0; i <= order; i++) {
        output += b_coeffs[i] * x_history[i];
    }
    for (int i = 1; i <= order; i++) {
        output -= a_coeffs[i] * y_history[i];
    }

    // Since Scipy's butterworth function returns coefficients with a[0] = 1,
    // we don't need to divide by a_coeffs[0]. If a[0] were not 1, we would
    // need to do: output /= a_coeffs[0];

    // Store the new output in the history and return it.
    y_history[0] = output;
    return output;
}