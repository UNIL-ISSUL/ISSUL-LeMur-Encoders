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

    print("const float b[] = {")
    for val in b:
        print(f"    {val:.10f},")
    print("};")

    print("const float a[] = {")
    for val in a:
        print(f"    {val:.10f},")
    print("};")
    print("\n")


if __name__ == "__main__":
    fs = 200.0  # Sampling frequency in Hz
    fc = 9.0    # Cutoff frequency in Hz

    # Generate and print coefficients for a 2nd-order filter
    order_2 = 2
    b2, a2 = generate_butter_coeffs(fs, fc, order_2)
    print_coeffs_for_cpp(b2, a2, order_2)

    # Generate and print coefficients for a 4th-order filter
    order_4 = 4
    b4, a4 = generate_butter_coeffs(fs, fc, order_4)
    print_coeffs_for_cpp(b4, a4, order_4)