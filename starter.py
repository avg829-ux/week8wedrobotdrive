"""CDS215 – Husky WASD obstacle-course challenge.

Run on a computer with a desktop GUI and PyBullet installed:

    pip install pybullet

    python CDS215_Husky_Obstacle_Course.py

Controls: W/S forward/backward, A/D turn; arrow keys also work.

Space stops, +/- changes speed, P reports pose, R resets, Q quits.

CHALLENGE: drive from the green START marker to the gold FINISH marker,

following the serpentine corridors without hitting the walls. A collision

counter tracks contacts with obstacle objects.

"""

import math

import time

import pybullet as p

import pybullet_data

# The wheel joint indices refer specifically to the PyBullet Husky URDF.

FRONT_LEFT_WHEEL = 2

FRONT_RIGHT_WHEEL = 3

REAR_LEFT_WHEEL = 4

REAR_RIGHT_WHEEL = 5

LEFT_WHEELS = [FRONT_LEFT_WHEEL, REAR_LEFT_WHEEL]

RIGHT_WHEELS = [FRONT_RIGHT_WHEEL, REAR_RIGHT_WHEEL]

# Coordinate system: X is the horizontal course direction, Y is vertical

# when viewed from above, and Z is height off the floor (in meters).

START_POSITION = [-8.0, -5.2, 0.2]

START_ANGLE = 0.0  # radians: robot initially faces +X

FINISH_XY = (8.0, 5.2)

GOAL_RADIUS = 0.85  # center-to-center distance needed to finish

# Course is a wide S-shaped lane from left/bottom to right/top.

# Perimeter: x [-10,10], y [-7,7]; interior baffles alternate gaps.

WALL_HEIGHT = 0.75

WALL_THICKNESS = 0.24

MOVEMENT_SPEED = 6.0

MINIMUM_SPEED = 1.0

MAXIMUM_SPEED = 15.0

def set_wheel_speeds(robot_id, left_speed, right_speed):

    """Command both left wheels and both right wheels in velocity mode.

    targetVelocity sets the requested joint rotation speed (rad/s).

    force is the maximum motor effort available to reach the target.

    """

    for wheel_joint in LEFT_WHEELS:

        p.setJointMotorControl2(

            bodyUniqueId=robot_id,

            jointIndex=wheel_joint,

            controlMode=p.VELOCITY_CONTROL,

            targetVelocity=left_speed,

            force=100,

        )

    for wheel_joint in RIGHT_WHEELS:

        p.setJointMotorControl2(

            bodyUniqueId=robot_id,

            jointIndex=wheel_joint,

            controlMode=p.VELOCITY_CONTROL,

            targetVelocity=right_speed,

            force=100,

        )

def move_forward(robot_id, speed):

    """Both sides rotate forward, pushing the robot ahead."""

    set_wheel_speeds(robot_id, speed, speed)

def move_backward(robot_id, speed):

    """Both sides reverse, pulling the robot backward."""

    set_wheel_speeds(robot_id, -speed, -speed)

def turn_left(robot_id, speed):

    """Left wheels reverse and right wheels advance: turn in place left."""

    set_wheel_speeds(robot_id, -speed, speed)

def turn_right(robot_id, speed):

    """Left wheels advance and right wheels reverse: turn in place right."""

    set_wheel_speeds(robot_id, speed, -speed)

def stop_robot(robot_id):

    """Request zero wheel velocities when no directional key is held."""

    set_wheel_speeds(robot_id, 0, 0)

def report_position(robot_id):

    """Print the robot's world position and heading (yaw, degrees)."""

    position, quaternion = p.getBasePositionAndOrientation(robot_id)

    yaw = p.getEulerFromQuaternion(quaternion)[2]

    print(f"Position: X={position[0]:.2f}, Y={position[1]:.2f}, "

          f"Z={position[2]:.2f}, heading={math.degrees(yaw):.0f}°")

def reset_robot(robot_id):

    """Return to START and cancel any leftover linear/angular momentum."""

    orientation = p.getQuaternionFromEuler([0, 0, START_ANGLE])

    p.resetBasePositionAndOrientation(robot_id, START_POSITION, orientation)

    p.resetBaseVelocity(robot_id,

                        linearVelocity=[0, 0, 0],

                        angularVelocity=[0, 0, 0])

    stop_robot(robot_id)

    print("Robot reset to START.")

def create_box(position, half_extents, color, mass=0):

    """Create a physical and visible box; mass=0 makes it stationary.

    half_extents are HALF the full width/length/height. A box with

    half_extents [0.5, 0.5, 0.5] is 1 m x 1 m x 1 m.

    RGBA color channels use values 0–1.

    """

    collision_shape = p.createCollisionShape(

        p.GEOM_BOX, halfExtents=half_extents)

    visual_shape = p.createVisualShape(

        p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)

    return p.createMultiBody(

        baseMass=mass,

        baseCollisionShapeIndex=collision_shape,

        baseVisualShapeIndex=visual_shape,

        basePosition=position,

    )

def key_is_down(keys, key_code):

    """True for a key currently HELD down; suitable for driving."""

    return (key_code in keys and

            bool(keys[key_code] & p.KEY_IS_DOWN))

def key_was_triggered(keys, key_code):

    """True for a NEW key press; suitable for one-time actions."""

    return (key_code in keys and

            bool(keys[key_code] & p.KEY_WAS_TRIGGERED))

def create_obstacle_course():

    """Build a walled S-shaped corridor plus small obstacles.

    Geometry plan (top view):

      • Perimeter walls form a 20 m × 14 m play area.

      • Baffle 1 at x=-4.5 extends from bottom to y=2.5;

        go around its TOP end.

      • Baffle 2 at x=0 extends from top to y=-2.5;

        go around its BOTTOM end.

      • Baffle 3 at x=4.5 extends from bottom to y=2.5;

        go around its TOP end.

    The passages are deliberately wide enough for the Husky to turn.

    """

    obstacle_ids = []

    wall_color = [0.35, 0.39, 0.45, 1]

    cone_color = [0.95, 0.30, 0.12, 1]

    def wall(x, y, hx, hy):

        obstacle_ids.append(create_box(

            [x, y, WALL_HEIGHT / 2],

            [hx, hy, WALL_HEIGHT / 2], wall_color))

    # Outside perimeter, four sides.

    wall(0, -7, 10, WALL_THICKNESS)

    wall(0, 7, 10, WALL_THICKNESS)

    wall(-10, 0, WALL_THICKNESS, 7)

    wall(10, 0, WALL_THICKNESS, 7)

    # Alternating interior barriers: navigate around open ends.

    wall(-4.5, -2.25, WALL_THICKNESS, 4.75)  # y: -7 to +2.5

    wall(0.0, 2.25, WALL_THICKNESS, 4.75)    # y: -2.5 to +7

    wall(4.5, -2.25, WALL_THICKNESS, 4.75)   # y: -7 to +2.5

    # Low boxes in broad corridor areas: drive around them.

    # These placements are away from the mandatory gaps at the baffles.

    small_obstacles = [

        (-7.1, -1.5, 0.38),

        (-2.3, 4.6, 0.32),

        (2.2, -4.7, 0.35),

        (7.0, 3.0, 0.34),

    ]

    for x, y, half_size in small_obstacles:

        obstacle_ids.append(create_box(

            [x, y, half_size],

            [half_size, half_size, half_size], cone_color))

    # Low colored pads are VISUAL markers, not collisions:

    # they deliberately sit just above the ground with no collision shape.

    def marker(x, y, rgba, width=0.85):

        shape = p.createVisualShape(

            p.GEOM_BOX,

            halfExtents=[width, width, 0.01],

            rgbaColor=rgba)

        p.createMultiBody(baseMass=0, baseVisualShapeIndex=shape,

                          basePosition=[x, y, 0.02])

    marker(START_POSITION[0], START_POSITION[1], [0.1, 0.8, 0.2, 1])

    marker(FINISH_XY[0], FINISH_XY[1], [1.0, 0.78, 0.08, 1])

    p.addUserDebugText("START", [START_POSITION[0] - 0.7, START_POSITION[1], 0.8],

                       textColorRGB=[0, 0.45, 0], textSize=1.4)

    p.addUserDebugText("FINISH", [FINISH_XY[0] - 0.7, FINISH_XY[1], 0.8],

                       textColorRGB=[0.7, 0.45, 0], textSize=1.4)

    return obstacle_ids

def reached_finish(robot_id):

    """True if the robot's X/Y location enters the finish circle."""

    position, _ = p.getBasePositionAndOrientation(robot_id)

    distance = math.hypot(position[0] - FINISH_XY[0],

                          position[1] - FINISH_XY[1])

    return distance < GOAL_RADIUS

def touching_obstacle(robot_id, obstacle_ids):

    """True when the robot is in contact with any wall or obstacle."""

    return any(p.getContactPoints(bodyA=robot_id, bodyB=obstacle_id)

               for obstacle_id in obstacle_ids)

# ------------------------------------------------------------

# SIMULATION SETUP

# ------------------------------------------------------------

client = p.connect(p.GUI)

if client < 0:

    raise RuntimeError("Could not open the PyBullet GUI.")

# Disable built-in visualization shortcuts so WASD controls only the Husky.
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.81)

p.setTimeStep(1 / 240)

p.loadURDF("plane.urdf")

start_orientation = p.getQuaternionFromEuler([0, 0, START_ANGLE])

robot_id = p.loadURDF("husky/husky.urdf", START_POSITION, start_orientation)

obstacle_ids = create_obstacle_course()

p.resetDebugVisualizerCamera(

    cameraDistance=23,

    cameraYaw=0,

    cameraPitch=-75,

    cameraTargetPosition=[0, 0, 0]

)

print("\nCDS215 HUSKY OBSTACLE COURSE")

print("Drive from GREEN START to GOLD FINISH.")

print("W/Up=forward, S/Down=backward, A/Left=turn left, D/Right=turn right")

print("Space=stop | +/-=speed | P=position | R=restart | Q=quit")

print("Click the PyBullet window before using the keyboard.\n")

movement_speed = MOVEMENT_SPEED

minimum_speed = MINIMUM_SPEED

maximum_speed = MAXIMUM_SPEED

running = True

collision_count = 0

was_touching = False

finished = False

start_time = time.monotonic()

while p.isConnected() and running:

    keys = p.getKeyboardEvents()

    # These variables are True when either WASD or the arrow key is held.
    forward_pressed = (key_is_down(keys, ord("w")) or key_is_down(keys, p.B3G_UP_ARROW))
   
    backward_pressed = (key_is_down(keys, ord("s")) or key_is_down(keys, p.B3G_DOWN_ARROW))

    left_pressed = (key_is_down(keys, ord("a")) or key_is_down(keys, p.B3G_LEFT_ARROW))

    right_pressed = (key_is_down(keys, ord("d")) or key_is_down(keys, p.B3G_RIGHT_ARROW))
       
    increase_speed = (key_is_down(keys, ord("+")) or key_is_down(keys, ord("=")))

    decrease_speed = key_is_down(keys, ord("-"))






    # BEGIN DECISION SECTION

    # Independent if statements permit more than one command per frame.
    if increase_speed:
        MOVEMENT_SPEED = min(movement_speed + 1, maximum_speed)
        print(f"Speed increased to {movement_speed}")

    if decrease_speed:
            MOVEMENT_SPEED = max(movement_speed - 1, minimum_speed)
            print(f"Speed decreased to {movement_speed}")   
    
    if key_was_triggered(keys, ord("p")):
        report_position(robot_id)

    if key_was_triggered(keys, ord("r")):
            reset_robot(robot_id)
            collision_count= 0
            was_touching= False
            finished = False
            start_time= time.monotonic()

    if key_was_triggered(keys, ord("q")):
            print("Quit keyboard")
            running = False
    



    # Only the FIRST matching movement branch runs: Space, W, S, A, D.

    if key_is_down(keys, ord(" ")):
        stop_robot(robot_id)
    elif forward_pressed:
        move_forward(robot_id, movement_speed)
    elif backward_pressed:
            move_backward(robot_id, movement_speed)
    elif left_pressed:
            turn_left(robot_id, movement_speed)
    elif right_pressed:
            turn_right(robot_id, movement_speed)
    else:
         stop_robot(robot_id)
            
            






    

    # END DECISION SECTION

    if not running:

        break

    # Provided obstacle-course tracking:

    p.stepSimulation()

    touching = touching_obstacle(robot_id, obstacle_ids)

    if touching and not was_touching:

        collision_count += 1

        print(f"Obstacle contact #{collision_count}")

    was_touching = touching

    if not finished and reached_finish(robot_id):

        finished = True

        stop_robot(robot_id)

        elapsed = time.monotonic() - start_time

        print(f"FINISH REACHED! Time: {elapsed:.1f} seconds; "

              f"obstacle contacts: {collision_count}")

        print("Press R to try again or Q to quit.")

    if finished:

        stop_robot(robot_id)

    time.sleep(1 / 240)

if p.isConnected():

    stop_robot(robot_id)

    p.disconnect()

print("Program ended.")
