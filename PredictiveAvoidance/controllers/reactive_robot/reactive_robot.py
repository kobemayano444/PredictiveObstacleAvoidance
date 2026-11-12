
from controller import Robot
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Motors
left_motor = robot.getDevice("left wheel")
right_motor = robot.getDevice("right wheel")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

max_speed = min(
    left_motor.getMaxVelocity(),
    right_motor.getMaxVelocity()
)

# Sensors
depth = robot.getDevice("kinect range")
gps = robot.getDevice("gps")
imu = robot.getDevice("imu")

depth.enable(timestep)
gps.enable(timestep)
imu.enable(timestep)

# These are controller and safety settings,
# not map coordinates or predefined routes.
FORWARD_SPEED = 2.2
DETOUR_SPEED = 1.5
SAFE_DISTANCE = 1.25
EMERGENCY_DISTANCE = 0.6

# Clearance for our robot's physical size
DETOUR_CLEARANCE = 0.9
PASS_MARGIN = 0.7

# Controller gains
LATERAL_GAIN = 1.5
HEADING_GAIN = 3.0
MAX_STEERING = 2.0

state = "TRACK"
detour_side = 1
target_offset = 0.0
obstacle_position = 0.0


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def set_motors(left, right):
    left_motor.setVelocity(
        clamp(left, -max_speed, max_speed)
    )
    right_motor.setVelocity(
        clamp(right, -max_speed, max_speed)
    )


def closest_distance(image, width, height, x1, x2):
    closest = float("inf")

    for y in range(height // 3, 2 * height // 3, 2):
        for x in range(x1, x2, 2):
            d = image[y * width + x]

            if math.isfinite(d) and d > 0:
                closest = min(closest, d)

    return closest


def read_environment():
    image = depth.getRangeImage()
    width = depth.getWidth()
    height = depth.getHeight()

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

    return left, center, right


# Obtain the first valid sensor measurements
if robot.step(timestep) == -1:
    raise SystemExit

# Save the ORIGINAL position and orientation
start_x, start_y = gps.getValues()[:2]
start_heading = imu.getRollPitchYaw()[2]

# Forward direction in Webots ENU coordinates
forward_x = math.cos(start_heading)
forward_y = math.sin(start_heading)

# Perpendicular direction to the robot's right
right_x = math.sin(start_heading)
right_y = -math.cos(start_heading)

count = 0

print("TRAJECTORY RECOVERY CONTROLLER ACTIVE")

while robot.step(timestep) != -1:

    left, center, right = read_environment()

    x, y = gps.getValues()[:2]
    heading = imu.getRollPitchYaw()[2]

    dx = x - start_x
    dy = y - start_y

    # Progress along the original direction
    progress = dx * forward_x + dy * forward_y

    # Sideways displacement from the original path
    offset = dx * right_x + dy * right_y

    # Obstacle appears on our original course
    if state == "TRACK" and center < SAFE_DISTANCE:

        detour_side = 1 if right > left else -1

        # Generate a local avoidance offset
        target_offset = (
            offset + detour_side * DETOUR_CLEARANCE
        )

        # Estimate obstacle position from perception
        obstacle_position = progress + center

        state = "DETOUR"

    elif state == "DETOUR":

        # Keep updating the obstacle estimate
        # if another close surface is visible
        if center < SAFE_DISTANCE:
            obstacle_position = max(
                obstacle_position,
                progress + center
            )

        # Move past the estimated obstacle position
        if progress > obstacle_position + PASS_MARGIN:
            state = "RETURN"

    elif state == "RETURN":

        # React again if the path is still blocked
        if center < SAFE_DISTANCE:
            detour_side = 1 if right > left else -1

            target_offset = (
                offset + detour_side * DETOUR_CLEARANCE
            )

            obstacle_position = progress + center
            state = "DETOUR"

        elif (
            abs(offset) < 0.12
            and abs(wrap_angle(start_heading - heading)) < 0.12
        ):
            state = "TRACK"

    # Determine the desired trajectory
    if state == "TRACK" or state == "RETURN":
        desired_offset = 0.0
    else:
        desired_offset = target_offset

    lateral_error = desired_offset - offset

    # Dynamically generate a steering heading
    
    desired_heading = (
        start_heading
        - math.atan(LATERAL_GAIN * lateral_error)
    )


    heading_error = wrap_angle(
        desired_heading - heading
    )

    correction = clamp(
        HEADING_GAIN * heading_error,
        -MAX_STEERING,
        MAX_STEERING
    )

    speed = (
        FORWARD_SPEED if state == "TRACK"
        else DETOUR_SPEED
    )

    # Emergency response to a very close obstacle
    if center < EMERGENCY_DISTANCE:
        speed = 0.0
        correction = -detour_side * MAX_STEERING

    set_motors(
        speed - correction,
        speed + correction
    )

    count += 1

    if count % 20 == 0:
        print(
            f"State={state} "
            f"Offset={offset:.2f}m "
            f"Depth={center:.2f}m "
            f"HeadingError={heading_error:.2f}"
        )

set_motors(0.0, 0.0)
