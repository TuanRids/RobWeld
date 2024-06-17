import random

def give_to_cpp():
    # initialize the value
    endvl = [
        [460, -200, 200, 180, -90, 0],
        [660, -100, 200, 180, -90, 0],
        [460, 0, 200, 180, -90, 0],
        [660, 100, 200, 180, -90, 0],
        [460, 200, 200, 180, -90, 0]
    ]
    # convert to float
    endvl_float = [[float(val) for val in sublist] for sublist in endvl]
    # return 2D array
    return endvl_float
