# Can be jpeg or png
string format
bool is_rgb

# Data has to be in range 0-255 to compressed
# To recover original data, original = scale*compressed + offset
float32 scale
float32 offset

uint8[] data
