#include "filter.h"
#include <string.h>

IIRFilter::IIRFilter() {
    this->order = 0;
    // Initialize coefficients and history arrays to zero
    memset(b_coeffs, 0, sizeof(b_coeffs));
    memset(a_coeffs, 0, sizeof(a_coeffs));
    memset(x_history, 0, sizeof(x_history));
    memset(y_history, 0, sizeof(y_history));
}

void IIRFilter::init(int order, const float* b, const float* a) {
    if (order > MAX_FILTER_ORDER) {
        // Handle error: filter order is too high for the defined maximum
        return;
    }

    this->order = order;

    // Copy coefficients, assuming a[0] is 1 and does not need to be stored.
    // The 'a' coefficients from python have a[0] = 1, so we use the rest.
    memcpy(b_coeffs, b, (order + 1) * sizeof(float));
    memcpy(a_coeffs, a, (order + 1) * sizeof(float));

    // Reset history
    memset(x_history, 0, sizeof(x_history));
    memset(y_history, 0, sizeof(y_history));
}

float IIRFilter::filter(float inputValue) {
    // Shift history
    for (int i = order; i > 0; i--) {
        x_history[i] = x_history[i - 1];
        y_history[i] = y_history[i - 1];
    }
    x_history[0] = inputValue;

    // Calculate output
    float output = 0.0;
    for (int i = 0; i <= order; i++) {
        output += b_coeffs[i] * x_history[i];
    }
    for (int i = 1; i <= order; i++) {
        output -= a_coeffs[i] * y_history[i];
    }

    // Scipy's butterworth function returns a[0] as 1, so we don't need to divide by it.
    // If a[0] were not 1, we would do: output /= a_coeffs[0];

    y_history[0] = output;
    return output;
}