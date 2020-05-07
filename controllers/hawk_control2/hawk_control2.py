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

hippo_radius = 12
hound_radius = 3

class Point: 
    def __init__(self, x, y): 
        self.x = x 
        self.y = y 
  
# Given three colinear points p, q, r, the function checks if  
# point q lies on line segment 'pr'  
def onSegment(p, q, r): 
    if ( (q.x <= max(p.x, r.x)) and (q.x >= min(p.x, r.x)) and 
           (q.y <= max(p.y, r.y)) and (q.y >= min(p.y, r.y))): 
        return True
    return False

#check orientation of the ordered tripplet  
def orientation(p, q, r): 
      
    val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y)) 
    if (val > 0): 
          
        # Clockwise orientation 
        return 1
    elif (val < 0): 
          
        # Counterclockwise orientation 
        return 2
    else: 
          
        # Colinear orientation 
        return 0

#function to check crossing between hippo and hound path. A path is from one node to another node.  
# The main function that returns true if the line segment 'p1q1' and 'p2q2' intersect. 
def doIntersect(p1,q1,p2,q2): 
      
    # Find the 4 orientations required for  
    # the general and special cases 
    o1 = orientation(p1, q1, p2) 
    o2 = orientation(p1, q1, q2) 
    o3 = orientation(p2, q2, p1) 
    o4 = orientation(p2, q2, q1) 
  
    # General case 
    if ((o1 != o2) and (o3 != o4)): 
        return True
  
    # Special Cases 
  
    # p1 , q1 and p2 are colinear and p2 lies on segment p1q1 
    if ((o1 == 0) and onSegment(p1, p2, q1)): 
        return True
  
    # p1 , q1 and q2 are colinear and q2 lies on segment p1q1 
    if ((o2 == 0) and onSegment(p1, q2, q1)): 
        return True
  
    # p2 , q2 and p1 are colinear and p1 lies on segment p2q2 
    if ((o3 == 0) and onSegment(p2, p1, q2)): 
        return True
  
    # p2 , q2 and q1 are colinear and q1 lies on segment p2q2 
    if ((o4 == 0) and onSegment(p2, q1, q2)): 
        return True
  
    # If none of the cases 
    return False

#p1 is from hound, p2 is from hippo.
#return true if for center p1 of hound, center p2 of hippo, then collide
def check_two_circles_intersect(p1,p2):
    if pow(hippo_radius - hound_radius,2) <= (pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2)) <= pow(hippo_radius - hound_radius,2):
        return true
    else:
        return false        

#it is a line segment
def check_line_intersect_circle(p1,p2,center,radius):
    ax = p1.x
    ay = p1.y
    bx = p1.x
    by = p1.y
    cx = center.x
    cy = center.y
    r = radius
    
    ax -= cx;
    ay -= cy;
    bx -= cx;
    by -= cy;
    a = (bx - ax)^2 + (by - ay)^2;
    b = 2*(ax*(bx - ax) + ay*(by - ay));
    c = ax^2 + ay^2 - r^2;
    disc = b^2 - 4*a*c;
    if(disc <= 0) return false;
    sqrtdisc = pow(disc,2);
    t1 = (-b + sqrtdisc)/(2*a);
    t2 = (-b - sqrtdisc)/(2*a);
    if((0 < t1 && t1 < 1) || (0 < t2 && t2 < 1)) return true;
    return false;

def swap(A,B):
    temp = B
    B.x = A.x
    B.y = A.y
    A.x = temp.x
    A.y = temp.y
    return A,B

#A,B are top, C,D are bottom
def pointInRectangle(P,A,B,C,D):
    if A.x > B.x:
        [A,B] = swap(A,B)                        
    if A.y < C.y:
        [A,C] = swap(A,C)
    if A.x <= P.x <= B.x and C.y <= P.y <= A.y:
        return true
    return false
         
def intersect(P,R,A,B,C,D):
    if pointInRectangle(P,A,B,C,D) or check_line_intersect_circle(A,B,P,R) or check_line_intersect_circle(B,C,P,R) or check_line_intersect_circle(C,D,P,R) or check_line_intersect_circle(D,A,P,R):
       return true
    return false 

#robot 1 is moving, while robot 2 is stationary. check if a rectangle overlap the circle or not.
def check_robot_is_moving(p1,p2,radius1,center,radius2):
    A = Point(p2.x - radius1, p2.y)
    B = Point(p2.x + radius1, p2.y)
    C = Point(p1.x - radius1, p1.y)
    D = Point(p1.x + radius1, p1.y)
    return intersect(center,radius2, A,B,C,D)
        


#function for stabalizing
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

corners = [(-2.75, -3), (2.5, -3), (2.5,3.5), (-2.75, 3.5)]

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

rende = (1.5 , 2.25)
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
        #initialize        
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
           

            #getting configspace for obstacles
            for cdx in range(centx-xdist,centx+xdist):
                for cdy in range(centy-ydist,centy+ydist):
                    if(cdx>=0 and cdx<camx and cdy>=0 and cdy<camy):
                        configSpace[cdx][cdy] = 1;
                        
        data = np.array(configSpace)
        data = np.transpose(data)
        #calling to map the RRT
        lab3.runRRT( 'exxample' ,1,data,[0,0],[camx-1,camy-1])
        
        #COLLISION AVOIDANCE CODE
        #Trajectories for Hound
        #Each node has the form of (x,y,delta,parent)
        hound_path = lab3.runRRT('hound',1,data,[0,0],[camx-1,camy-1])
        #Trajectories for Hippo
        hippo_path = lab3.runRRT('hippo',1,data,[0,0],[camx-1,camy-1])
        
        #Translating from pixels to real worls unit
        
        #Translating to worlds coordinates
        
        #x,y,delta,parent,Fifth field is a list containing potential crush [x,y],6th field is a list containing static collision between hippo and hound when they stay at their nodes
        #7th field is to store the potential nodes of colliding while driving
        for i in range(len(hound_path)):
            potential_crushing_nodes = []
            hound_path[i].append(potential_crushing_nodes)
            colliding_nodes = []
            hound_path[i].append(colliding_nodes)
            hound_path[i].append(colliding_nodes)
            
        for i in range(len(hippo_path)):
            potential_crushing_nodes = []
            hippo_path[i].append(potential_crushing_nodes)        
            colliding_nodes = []
            hippo_path[i].append(colliding_nodes)
            hippo_path[i].append(colliding_nodes)
            
        #Four cases to check for
        #checking for intersecting paths between the two trajectories and mark the path.
        for i in range(len(hound_path) -1):
            for j in range(len(hippo_path) -1):
                #this function takes in 8 arguments, x intial and y intial from hound, x final and y final from hound; x intial and y intial from hippo, x final and y final from hippo
                init_hound = Point(hound_path[i][0], hound_path[i][1])
                final_hound = Point(hound_path[i+1][0], hound_path[i+1][1])
                init_hippo = Point(hippo_path[j][0], hippo_path[j][1])
                final_hippo = Point(hippo_path[j+1][0], hippo_path[j+1][1])                
                if doIntersect(init_hound,final_hound, init_hippo, final_hippo):
                    #append the potential crushing node of hippo, there could be more than one potential crushing node
                    hound_path[i][4].append([init_hippo.x, init_hippo.y])
                    #append the potential crushing node of hound
                    hippo_path[j][4].append([init_hound.x, init_hound.y])
                if check_two_circles_intersect(init_hound,init_hippo):
                    #two circles intersect for that state, share each other information, to check for during execution
                    hound_path[i][5].append([init_hippo.x,init_hippo.y])
                    hippo_path[j][5].append([init_hound.x,init_hound.y])
                if check_robot_is_moving(init_hound,final_hound, hound_radius, init_hippo, hippo_radius):
                    #hound is moving from inital node to final node, check if hippo in j node can cause collision
                    hound_path[i][6].append([init_hippo.x, init_hippo.y])
                if check_robot_is_moving(init_hippo,final_hippo,hippo_radius, init_hound, hound_radius):
                    #hippo is moving from inital node to final node, check if hound in i node can cause collision                      
                    hippo_path[j][6].append([init_hound.x, init_hound.y])

    #send the hound and hippo path to hound and hippo.        
        
                
        
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
