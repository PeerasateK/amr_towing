from math import *
regions = {
        'right':  3,
        'fright': 5,
        'front':  9,
        'fleft':  8,
        'left':   6,
    }

print(max(regions, key=regions.get))