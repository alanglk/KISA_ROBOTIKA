
#!/usr/bin/env python
from math import exp, pi, fmod
def normalize_angle(angle):
    """
    Wrap the angle between -pi and pi.
    """
    pi_2 = 2. * pi

    a = fmod(fmod(angle, pi_2) + pi_2, pi_2) 
    if a > pi:
        a -= 2. * pi
    return a

def sigmoid(x):
    """Numerically-stable sigmoid function."""
    if x >= 0:
        z = exp(-x)
        return 1 / (1 + z)
    else:
        # if x is less than zero then z will be small, denom can't be
        # zero because it's 1+z.
        z = exp(x)
        return z / (1 + z)

 