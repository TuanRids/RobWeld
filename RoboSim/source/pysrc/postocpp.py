import random

def give_to_cpp():
    endvl = []
    
    for i in range (0,5):
        values = []
        for _ in range(3):
            values.append(random.uniform(-300, 300))
            # 3 số sau ngẫu nhiên từ -180 đến 180
        for _ in range(3):
            values.append(random.uniform(-180, 180))
        endvl.append(values)
    return endvl
