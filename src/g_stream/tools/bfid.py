import numpy as np


def binary_array_to_int(binary_array):
    """Convert a list of bits (MSB first) to an integer."""
    value = 0
    for bit in binary_array:
        value = (value << 1) | bit
    return value

def extract_binary_from_image(image, start_x=0, start_y=0, bit_size=3, total_bits=64):
    """
    Extracts 64 bits from the top-left of the image.
    Each bit is represented by a 3x3 pixel block (white=1, black=0).
    """
    bits = []
    for i in range(total_bits):
        x = start_x + (i % 32) * bit_size
        y = start_y + (i // 32) * bit_size

        roi = image[y:y + bit_size, x:x + bit_size]  # Extract the bit region
        avg_color = np.mean(roi)  # Compute the average brightness
        bit = 1 if avg_color > 128 else 0  # Threshold to classify white or black
        bits.append(bit)

    # Convert to integer values
    sec = binary_array_to_int(bits[:32])  # First 32 bits for sec (int32)
    nanosec = binary_array_to_int(bits[32:])  # Last 32 bits for nanosec (uint32)

    return sec, nanosec