"""Random utility functions that can be used"""

def _mark_unused(func):
    """Decorator to mark a function as currently unsued, but kept for later utility"""
    def wrapper(*args, **kwargs):
        print(f"\033[31mWarning: Function {func.__name__!r} is marked as unused, please update the documentation.\033[0m")
        return func(*args, **kwargs)
    return wrapper
