"""reactive_robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor

from controller import Robot
import math

# ==========================================
# 1. INITIALIZE THE ROBOT
# ==========================================

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Get the wheel motors
left_motor = robot.getDevice("left wheel")
right_motor = robot.getDevice("right wheel")

# Enable continuous rotation
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Get the Kinect sensors
camera = robot.getDevice("kinect color")
depth = robot.getDevice("kinect range")

camera.enable(timestep)
depth.enable(timestep)


# ==========================================
# 2. ROBOT SETTINGS
# ==========================================

# Respect the motor limits
max_speed = min(
    left_motor.getMaxVelocity(),
    right_motor.getMaxVelocity()
)

FORWARD_SPEED = min(2.5, max_speed * 0.35)
TURN_SPEED = min(1.6, max_speed * 0.25)

# Distances are measured in meters
OBSTACLE_DISTANCE = 1.3
CLEAR_DISTANCE = 1.7
SIDE_DANGER_DISTANCE = 0.65
SIDE_CLEAR_DISTANCE = 0.9

# Remember the current maneuver
# -1 = turning left
#  0 = going forward
#  1 = turning right
turn_direction = 0

clear_readings = 0
count = 0


# ==========================================
# 3. DEPTH SENSOR PROCESSING
# ==========================================

def closest_distance(image, width, height,
                     start_x, end_x):

    closest = float("inf")

    for y in range(height // 3,
                   2 * height // 3, 2):

        for x in range(start_x, end_x, 2):

            distance = image[y * width + x]

            if math.isfinite(distance) and distance > 0:
                closest = min(closest, distance)

    return closest


def read_environment():

    image = depth.getRangeImage()

    width = depth.getWidth()
    height = depth.getHeight()

    left = closest_distance(
        image, width, height,
        0, width // 3
    )

    center = closest_distance(
        image, width, height,
        width // 3, 2 * width // 3
    )

    right = closest_distance(
        image, width, height,
        2 * width // 3, width
    )

    return left, center, right


# ==========================================
# 4. MOTOR CONTROL
# ==========================================

def move_forward():
    left_motor.setVelocity(FORWARD_SPEED)
    right_motor.setVelocity(FORWARD_SPEED)


def turn_left():
    left_motor.setVelocity(-TURN_SPEED)
    right_motor.setVelocity(TURN_SPEED)


def turn_right():
    left_motor.setVelocity(TURN_SPEED)
    right_motor.setVelocity(-TURN_SPEED)


def stop_robot():
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)


# ==========================================
# 5. MAIN REACTIVE NAVIGATION
# ==========================================

print("REACTIVE OBSTACLE AVOIDANCE ACTIVE")

while robot.step(timestep) != -1:

    left, center, right = read_environment()

    count += 1

    # Detect nearby obstacles
    obstacle_ahead = center < OBSTACLE_DISTANCE

    side_danger = (
        left < SIDE_DANGER_DISTANCE or
        right < SIDE_DANGER_DISTANCE
    )

    # Choose an avoidance direction
    if turn_direction == 0:

        if obstacle_ahead or side_danger:

            if left > right:
                turn_direction = -1
            else:
                turn_direction = 1

            clear_readings = 0

    # If turning, check whether we have
    # enough visible clearance to resume
    if turn_direction != 0:

        path_clear = (
            center > CLEAR_DISTANCE and
            left > SIDE_CLEAR_DISTANCE and
            right > SIDE_CLEAR_DISTANCE
        )

        if path_clear:
            clear_readings += 1
        else:
            clear_readings = 0

        # Avoid changing direction after
        # just one sensor reading
        if clear_readings >= 5:
            turn_direction = 0
            clear_readings = 0

    # Execute the chosen behavior
    if turn_direction == -1:
        turn_left()
        action = "TURN LEFT"

    elif turn_direction == 1:
        turn_right()
        action = "TURN RIGHT"

    else:
        move_forward()
        action = "FORWARD"

    # Print useful debugging information
    if count % 20 == 0:

        print(
            f"L={left:.2f}m "
            f"C={center:.2f}m "
            f"R={right:.2f}m "
            f"Action={action}"
        )

stop_robot()
