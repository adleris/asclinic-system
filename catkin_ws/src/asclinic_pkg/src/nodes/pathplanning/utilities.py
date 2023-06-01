"""Random utility functions that can be used"""

import numpy as np

def _mark_unused(func):
    """Decorator to mark a function as currently unsued, but kept for later utility"""
    def wrapper(*args, **kwargs):
        print(f"\033[31mWarning: Function {func.__name__!r} is marked as unused, please update the documentation.\033[0m")
        return func(*args, **kwargs)
    return wrapper

def point_distance(p1: Point, p2: Point):
    """Calculates the distances between two Point objects
    :param p1: Point 1
    :param p2: Point 2"""
    return np.sqrt( (p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2 )
