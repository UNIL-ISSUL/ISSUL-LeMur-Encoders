import scipy.signal

def generate_butter_coeffs(fs, fc, order):
    """
    Generates IIR Butterworth low-pass filter coefficients.

    Args:
        fs (float): Sampling frequency in Hz.
        fc (float): Cutoff frequency in Hz.
        order (int): Filter order.

    Returns:
        tuple: A tuple containing the numerator (b) and denominator (a)
               coefficients of the filter.
    """
    nyquist = 0.5 * fs
    normal_cutoff = fc / nyquist
    b, a = scipy.signal.butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def print_coeffs_for_cpp(b, a, order):
    """
    Prints filter coefficients in a C++-friendly format.

    Args:
        b (array-like): Numerator coefficients.
        a (array-like): Denominator coefficients.
        order (int): Filter order.
    """
    print(f"// Butterworth Low-pass Filter, Order = {order}")
    print(f"// Sampling Frequency (fs) = {fs} Hz, Cutoff Frequency (fc) = {fc} Hz")
    
    print(f"const int filter_order = {order};")

    print("const float b_coeffs[] = {")
    for val in b:
        print(f"    {val:.10f},")
    print("};")

    print("const float a_coeffs[] = {")
    for val in a:
        print(f"    {val:.10f},")
    print("};")
    print("\n")


if __name__ == "__main__":
    fs = 200.0  # Sampling frequency in Hz
    fc = 3.0    # Cutoff frequency in Hz

    # Generate and print coefficients for a 4th-order filter
    order = 3
    b, a = generate_butter_coeffs(fs, fc, order)
    print_coeffs_for_cpp(b, a, order)
