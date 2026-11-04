"""predictive_robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math

print("THREE-REGION SENSOR TEST ACTIVE")
# Initialize the robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Access the motors
left_motor = robot.getDevice("left wheel")
right_motor = robot.getDevice("right wheel")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Access the Kinect
camera = robot.getDevice("kinect color")
depth = robot.getDevice("kinect range")

camera.enable(timestep)
depth.enable(timestep)

# Access one ultrasonic sensor
sonar = robot.getDevice("so3")
sonar.enable(timestep)


# Find the closest valid distance in a region
def closest_distance(image, width, height, start_x, end_x):
    closest = float("inf")

    # Examine the middle portion of the image
    for y in range(height // 3, 2 * height // 3):
        for x in range(start_x, end_x):

            distance = image[y * width + x]

            if math.isfinite(distance) and distance > 0:
                if distance < closest:
                    closest = distance

    return closest


count = 0

while robot.step(timestep) != -1:

    count += 1

    # Display readings every 20 steps
    if count % 20 == 0:

        image = depth.getRangeImage()
        width = depth.getWidth()
        height = depth.getHeight()

        # Split the image into three regions
        left = closest_distance(
            image, width, height, 0, width // 3
        )

        center = closest_distance(
            image, width, height,
            width // 3, 2 * width // 3
        )

        right = closest_distance(
            image, width, height,
            2 * width // 3, width
        )

        print("LEFT:  ", round(left, 2), "m")
        print("CENTER:", round(center, 2), "m")
        print("RIGHT: ", round(right, 2), "m")
        print("SONAR: ", round(sonar.getValue(), 2))
        print("--------------------")

