#ifndef IIR_FILTER_H
#define IIR_FILTER_H

#define MAX_FILTER_ORDER 6

/**
 * @class IIRFilter
 * @brief A class for implementing a single IIR filter.
 *
 * This class provides a simple interface for initializing and running an IIR filter.
 * It supports filters up to a maximum order defined by MAX_FILTER_ORDER.
 */
class IIRFilter {
public:
    /**
     * @brief Construct a new IIRFilter object.
     */
    IIRFilter();

    /**
     * @brief Initializes the IIR filter with given coefficients.
     * @param order The order of the filter.
     * @param b A pointer to an array of numerator coefficients.
     * @param a A pointer to an array of denominator coefficients.
     */
    void init(int order, const float* b, const float* a);

    /**
     * @brief Applies the filter to a new input value.
     * @param inputValue The new value to be filtered.
     * @return The filtered output value.
     */
    float filter(float inputValue);

private:
    int order; ///< The order of the filter.
    float b_coeffs[MAX_FILTER_ORDER + 1]; ///< The numerator coefficients.
    float a_coeffs[MAX_FILTER_ORDER + 1]; ///< The denominator coefficients.
    float x_history[MAX_FILTER_ORDER + 1]; ///< The history of input values.
    float y_history[MAX_FILTER_ORDER + 1]; ///< The history of output values.
};

#endif // IIR_FILTER_H