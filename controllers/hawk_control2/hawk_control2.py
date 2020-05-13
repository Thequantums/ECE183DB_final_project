"""hawk_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import ctypes
from matplotlib import pyplot as plt
import numpy as np
from controller import Robot,Camera,CameraRecognitionObject,Compass,GPS,Gyro,InertialUnit,Keyboard,LED,Motor
import lab3
from scipy.spatial import distance
import collision_geometry as cg

hound_radius = 1 #should be in meter
hippo_radius = 0.25 #should be in meter

def get_goal_state(corners, fov):
    length = distance.euclidean(corners[0], corners[1])
    width = distance.euclidean(corners[0], corners[3])
    m = max(width, length)
    altitude = .9*m / (2*np.arctan(1/2*fov))
    altitude = round(altitude, 2)
    x = .5*(corners[2][0] + corners[0][0])
    z = .5*(corners[2][1] + corners[0][1])
    run = corners[1][0] - corners[0][0]
    rise = corners[1][1] - corners[0][1]
    if run == 0:
        yaw = np.pi/2
    else:
        yaw = np.tan(rise/run)
    return [x, altitude, z, yaw]

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
camera = robot.getCamera("camera")
camera.enable(timestep)
camera.recognitionEnable(timestep)
front_left_led = robot.getLED("front left led")
front_right_led = robot.getLED("front right led")
imu = robot.getInertialUnit("inertial unit")
imu.enable(timestep)
gps = robot.getGPS("gps")
gps.enable(timestep)
compass = robot.getCompass("compass")
compass.enable(timestep)
gyro = robot.getGyro("gyro")
gyro.enable(timestep)
camera_roll_motor = robot.getMotor("camera roll");
camera_pitch_motor = robot.getMotor("camera pitch");


front_left_motor = robot.getMotor("front left propeller");
front_right_motor = robot.getMotor("front right propeller");
rear_left_motor = robot.getMotor("rear left propeller");
rear_right_motor = robot.getMotor("rear right propeller");
motors = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};

corners = [(-1.5, -.5), (4, -.5), (4,3), (-1, 3)]

hover_zone = get_goal_state(corners, 1)

print(hover_zone)

for m in motors:
    m.setPosition(math.inf);
    m.setVelocity(1.0);

while (robot.step(timestep) != -1):
    if (robot.getTime() > 1.0):
        break

print("Start the drone...");

#print("You can control the drone with your computer keyboard:")
#print("- 'up': move forward.")
#print("- 'down': move backward.")
#print("- 'right': turn right.")
#print("- 'left': turn left.")
#print("- 'shift + up': increase the target altitude.")
#print("- 'shift + down': decrease the target altitude.")
#print("- 'shift + right': strafe right.")
#print("- 'shift + left': strafe left.")

roll_trim = 0.07 #positive is right
pitch_trim = 0.15 #positive is backwards
k_vertical_thrust = 68.5
k_vertical_offset = 0.6
k_vertical_p = 3.0
k_roll_p = 50.0
k_pitch_p = 30.0
killswitch = 0.0

target_yaw = imu.getRollPitchYaw()[2] +.02

target_altitude = hover_zone[1]
target_x = hover_zone[0]
target_z = hover_zone[2]

camx=camera.getWidth()
camy=camera.getHeight()
configSpace = []

for k in range(camx):
    configtemp = []
    for l in range(camy):
        configtemp.append(0)
    configSpace.append(configtemp)

latch = False

rende = (4 , 3)
request = False
hover1 = 0
hover_mode = False

x1 = 0
x_good = False

z1 = 0
z_good = False

done = False


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1 and killswitch != 1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    time = robot.getTime()



    roll = imu.getRollPitchYaw()[0] + (math.pi / 2.0)
    pitch = imu.getRollPitchYaw()[1]
    yaw = imu.getRollPitchYaw()[2]
    altitude = gps.getValues()[1]
    x = gps.getValues()[0]
    z = gps.getValues()[2]
    roll_acceleration = gyro.getValues()[0]
    pitch_acceleration = gyro.getValues()[1]

    x = gps.getValues()[0]


    led_state = int(time) % 2
    front_left_led.set(led_state)
    front_right_led.set(~led_state)


    camera_roll_motor.setPosition( -0.115 * roll_acceleration)
    camera_pitch_motor.setPosition(-0.1 * pitch_acceleration)
    compVal = compass.getValues()
    heading = ((math.atan2(compVal[0],compVal[1]))+math.pi)*(180/math.pi)

    roll_disturbance = 0.0
    pitch_disturbance = 0.0
    yaw_disturbance = 0.0

    altitude_error = abs(altitude - target_altitude)

    if altitude_error < .4 and not hover_mode:
        hover1 = hover1 + 1
        if hover1 > 100:
            hover_mode = True
            print('Achieved altitude')
    elif altitude_error >= .5:
        hover1 = 0
        hover_mode = False

    if done and hover_mode and x_good and z_good:
        killswitch = 1

    x_error = target_x - x
    if hover_mode:
        if abs(x_error) < .1 and not x_good:
            x1 = x1 + 1
            if x1 > 100:
                x_good = True
                print('Achieved x')
        elif abs(x_error) >= .1:
            x1 = 0
            x_good = False
            pitch_disturbance = max(min(x_error, 2), -2)


    z_error = target_z - z
    if hover_mode and x_good:
        if abs(z_error) < .04 and not z_good:
            z1 = z1 + 1
            if z1 > 75:
                print('Achieved z')
                z_good = True
                request = True
        elif abs(z_error) >= .04:
            z1 = 0
            z_good = False
            roll_disturbance = -max(min(z_error, 1.5), -1.5)

    if request and not done:
        number_of_objects = camera.getRecognitionNumberOfObjects();
        #print("\n Recognized ",number_of_objects, " objects." );
        if number_of_objects > 0:
                objects = camera.getRecognitionObjects()

        for k in range(camx):
            for l in range(camy):
                configSpace[k][l] = 0

       #using webots super camera
        for i in objects:
            pptr = int(i.position)
            position = ctypes.c_double * 3
            position = position.from_address(pptr)
            optr = int(i.orientation)
            orientation = ctypes.c_double * 4
            orientation = orientation.from_address(optr)
            piptr = int(i.position_on_image)
            position_on_image = ctypes.c_int * 2
            position_on_image = position_on_image.from_address(piptr)
            siptr = int(i.size_on_image)
            size_on_image = ctypes.c_int * 2
            size_on_image = size_on_image.from_address(siptr)
            #print("Model of object ", i.model);
            #print("Camera: ", camx," ", camy);
            #print("Relative position of object ", i.model, position[0],position[1],position[2])
            #print("Relative orientation of object \n", i.model, orientation[0],orientation[1],orientation[2],orientation[3])
            #print("Position of the object %d on the camera image: ", i.model, position_on_image[0],position_on_image[1])
            #print("Size of the object %d on the camera image: ", i.model, size_on_image[0],size_on_image[1])
            centx =position_on_image[0]
            centy = position_on_image[1]
            xdist = math.ceil(size_on_image[0]/2)
            ydist = math.ceil(size_on_image[1]/2)
            print(i.get_colors())
            if(i.get_colors() == [1,1,1]):
                houndstart = [centx,camy - centy]
            if (i.get_colors() == [0, 0, 0]):
                hippostart = [centx,camy - centy]
            if(i.get_colors() == [1,0,0]):
                #getting configspace for obstacles
                for cdx in range(centx-xdist,centx+xdist):
                    for cdy in range(centy-ydist,centy+ydist):
                        if(cdx>=0 and cdx<camx and cdy>=0 and cdy<camy):
                            configSpace[cdx][cdy] = 1;

        data = np.array(configSpace)
        data = np.transpose(data)
        #calling to map the RRT
        path = lab3.runRRT( 'exxample' ,1, data,houndstart,[400,400])
        path = lab3.runRRT('exxample', 1, data, hippostart, [400,400])
        print(path)
        if False:
            # COLLISION AVOIDANCE CODE
            # Trajectories for Hound
            # Each node has the form of (x,y,delta,parent)
            hound_path = lab3.runRRT('hound', 1, data, [0, 0], [camx - 1, camy - 1])
            # Trajectories for Hippo
            hippo_path = lab3.runRRT('hippo', 1, data, [0, 0], [camx - 1, camy - 1])

            # Translating from pixels to real worls unit

            # Translating to worlds coordinates

            # each node has the form of: x, y, delta, parent, type A conflicts nodes, type B conflict nodes, type C conflict nodes.
            #Type A: attemp_to_go_conflict_node (Crossing path) (list)
            #Type B: driving conflict node (list)
            #Type C: arriving conflict node (list)
            # 7th field is to store the potential nodes of colliding while driving
            #Append three empty lists to each nodes. The three lists are for storing type A, type B, type C conflicts nodes
            for i in range(len(hound_path)):
                hound_path[i].extend(([],[],[]))

            for i in range(len(hippo_path)):
                hippo_path[i].extend(([],[],[]))

            #Mark the conflict nodes and exchange information between hound and hippo
            for i in range(len(hound_path) - 1):
                init_hound = cg.Point(hound_path[i][0], hound_path[i][1]) #point of current node of hound
                final_hound = cg.Point(hound_path[i + 1][0], hound_path[i + 1][1]) #point of next node of hound
                for j in range(len(hippo_path) - 1):
                    init_hippo = cg.Point(hippo_path[j][0], hippo_path[j][1]) #point of current node of hippo
                    final_hippo = cg.Point(hippo_path[j + 1][0], hippo_path[j + 1][1]) #point of next node of hippo
                    if cg.doIntersect(init_hound, final_hound, init_hippo, final_hippo):
                        # paths crossed between hound and hippo. This create a type A conflict
                        hound_path[i][4].append([init_hippo.x, init_hippo.y])
                        hippo_path[j][4].append([init_hound.x, init_hound.y])
                    if cg.check_two_circles_intersect(init_hound,hound_radius,init_hippo,hippo_radius):
                        # if a hound and a hippo occupied a node with their respective radii, then it overlaps
                        hound_path[i][5].append([init_hippo.x, init_hippo.y])
                        hippo_path[j][5].append([init_hound.x, init_hound.y])
                    if cg.check_robot_is_moving(init_hound, final_hound, hound_radius, init_hippo, hippo_radius):
                        # hound is moving from inital node to next node, check if hippo is stationary in j node can overlap the path hound is moving
                        hound_path[i][6].append([init_hippo.x, init_hippo.y])
                    if cg.check_robot_is_moving(init_hippo, final_hippo, hippo_radius, init_hound, hound_radius):
                        # hippo is moving from inital node to next node, check if hound is stationary in i node can overlap the path hippo is moving
                        hippo_path[j][6].append([init_hound.x, init_hound.y])

            #check type B conflicts for goal node of hound and hippo
            hound_goal_node = cg.Point(hound_path[len(hound_path) -1].x,hound_path[len(hound_path) -1].y)
            hippo_goal_node = cg.Point(hippo_path[len(hippo_path) -1].x, hippo_path[len(hippo_path) -1].y)
            for i in range(len(hippo_path)):
                init_hippo = cg.Point(hippo_path[i][0], hippo_path[i][1]) #point of current node of hippo
                if cg.doIntersect(hound_goal_node,hound_radius,init_hippo,hippo_radius):
                    hound_path[len(hound_path) - 1][5].append([init_hippo.x, init_hippo.y])
            for i in range(len(hound_path)):
                init_hound = cg.Point(hound_path[i][0], hound_path[i][1])
                if cg.doIntersect(hippo_goal_node, hippo_radius, init_hound, hound_radius):
                    hippo_path[len(hippo_path) - 1][5].append([init_hound.x, init_hound.y])

            # send the hound and hippo path to hound and hippo.

            request = 0
            done = True
            x_good = False
            z_good = False
            target_x = rende[0]
            target_z = rende[1]

    if done and x_good and z_good:
        target_altitude = 1
        hover_mode = False

    clamped_difference_yaw = max(min(target_yaw - yaw, 1), -1)
    yaw_input = -clamped_difference_yaw
    roll_input = k_roll_p * max(min(roll, 1), -1) + roll_acceleration + roll_disturbance + roll_trim
    pitch_input = k_pitch_p * max(min(pitch, 1), -1) - pitch_acceleration + pitch_disturbance - pitch_trim
    clamped_difference_altitude = max(min(target_altitude - altitude + k_vertical_offset, 1), -1)
    vertical_input = k_vertical_p * math.pow(clamped_difference_altitude, 3.0)


    front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
    front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
    rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
    rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input

    if(killswitch == 1):
        front_left_motor_input = 0.0;
        front_right_motor_input = 0.0;
        rear_left_motor_input = 0.0;
        rear_right_motor_input = 0.0;


    front_left_motor.setVelocity(front_left_motor_input)
    front_right_motor.setVelocity(-front_right_motor_input)
    rear_left_motor.setVelocity(-rear_left_motor_input)
    rear_right_motor.setVelocity(rear_right_motor_input)





    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)


# Enter here exit cleanup code.
