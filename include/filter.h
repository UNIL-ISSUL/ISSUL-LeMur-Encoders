#ifndef IIR_FILTER_H
#define IIR_FILTER_H

#define MAX_FILTER_ORDER 6

class IIRFilter {
public:
    IIRFilter();
    void init(int order, const float* b, const float* a);
    float filter(float inputValue);

private:
    int order;
    float b_coeffs[MAX_FILTER_ORDER + 1];
    float a_coeffs[MAX_FILTER_ORDER + 1];
    float x_history[MAX_FILTER_ORDER + 1]; // Input history
    float y_history[MAX_FILTER_ORDER + 1]; // Output history
};

#endif // IIR_FILTER_H