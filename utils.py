import random

# List of cone colors used in the project
COLORS = ["blue", "green", "red", "orange", "yellow"]

# Area threshold of the bounding box (in pixels) to consider the cone "reached"
AREA_THRESHOLD = 60000

# Safety limit to avoid infinite loops while searching for a color
MAX_ITERATIONS = 10000


def get_randomized_colors() -> list:
    """
    Return a shuffled list of cone colors.
    """
    colors = COLORS.copy()
    random.shuffle(colors)
    return colors
