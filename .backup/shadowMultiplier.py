import math

def calculate_shadow_multiplier(avg):
    # Define parameters for the non-linear function
    a = 0.1  # Minimum multiplier for the darkest effect
    b = 0.8  # Maximum multiplier for the lightest effect
    k = 5.0  # Steepness of the transition

    # Use a logistic function to calculate the multiplier
    multiplier = a + (b - a) / (1 + math.exp(-k * ((avg / 255) - 0.5)))
    return 1 - multiplier

# Test the function
for avg in [0, 50, 100, 150, 200, 255]:
    print(f"Avg: {avg}, Multiplier: {calculate_shadow_multiplier(avg):.3f}")
