import random


def get_random():
    number = random.uniform(0.0,10.0)
    return f"{number:.1f}"


for i in range(15):
    print(f"Pose2D(x={get_random()}, y={get_random()})")
