def int_to_binary_array(value, bits):
        """Convert an integer to a list of bits (MSB first)."""
        return [(value >> i) & 1 for i in range(bits - 1, -1, -1)]

def draw_binary_on_image(image, sec, nanosec, start_x=0, start_y=0, bit_size=3):
    """
    Draws binary representation of sec (int32) and nanosec (uint32) on the image using NumPy.
    Each bit is represented as a `bit_size x bit_size` pixel block.
    """
    sec_bits = int_to_binary_array(sec, 32)
    nanosec_bits = int_to_binary_array(nanosec, 32)
    bits = sec_bits + nanosec_bits  # Concatenate both numbers (64 bits total)

    for i, bit in enumerate(bits):
        x = start_x + (i % 32) * bit_size
        y = start_y + (i // 32) * bit_size
        color = 255 if bit == 1 else 0  # White (1) or Black (0)

        # Set pixel values using NumPy slicing
        image[y:y + bit_size, x:x + bit_size] = color

    return image