/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Starts with a predefined behaviors and then
 *                read the user keyboard inputs to actuate the
 *                robot
 */

#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/types.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>

//plateform_speed is the speed of the body in m/s for SPEED define as 4.0 bellow
#define plateform_speed_vx 0.1362
#define plateform_speed_vy 0.1977
#define SPEED 4.0
#define DISTANCE_TOLERANCE 0.02
#define ANGLE_TOLERANCE 0.02

// stimulus coefficients

#define K1 80.0
#define K2 1.0
#define K3 1.0


#define TIME_STEP 32


//============code for tiny math===========================

typedef struct {
  double u;
  double v;
} Vector2;

typedef struct {
  double u;
  double v;
  double w;
} Vector3;

typedef struct {
  Vector3 a;
  Vector3 b;
  Vector3 c;
} Matrix33;

void vector3_set_values(Vector3 *vect, double u, double v, double w) {
  vect->u = u;
  vect->v = v;
  vect->w = w;
}

void matrix33_set_values(Matrix33 *m, double au, double av, double aw, double bu, double bv, double bw, double cu, double cv,
                         double cw) {
  vector3_set_values(&(m->a), au, av, aw);
  vector3_set_values(&(m->b), bu, bv, bw);
  vector3_set_values(&(m->c), cu, cv, cw);
}

void matrix33_set_identity(Matrix33 *m) {
  matrix33_set_values(m, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
}

void matrix33_mult_vector3(Vector3 *res, const Matrix33 *m, const Vector3 *v) {
  res->u = m->a.u * v->u + m->b.u * v->v + m->c.u * v->w;
  res->v = m->a.v * v->u + m->b.v * v->v + m->c.v * v->w;
  res->w = m->a.w * v->u + m->b.w * v->v + m->c.w * v->w;
}

double vector2_norm(const Vector2 *v) {
  return sqrt(v->u * v->u + v->v * v->v);
}

void vector2_minus(Vector2 *v, const Vector2 *v1, const Vector2 *v2) {
  v->u = v1->u - v2->u;
  v->v = v1->v - v2->v;
}

double vector2_angle(const Vector2 *v1, const Vector2 *v2) {
  return atan2(v2->v, v2->u) - atan2(v1->v, v1->u);
}

double bound(double v, double a, double b) {
  return (v > b) ? b : (v < a) ? a : v;
}


//===========code for drive function================
// if lock is 0, it is time to translate, if lock is 1. it's time to rotate.
//if lock is 2, it's time to stop.
int lock = 0;
int node_num = 0;

typedef struct {
  Vector2 v_target;
  double alpha;
  bool reached;
  double time;
  double time_to_go;
} goto_struct;

typedef struct {
  double x;
  double z;
  double alpha;
} node;

//x,y,delta,parent,Fifth field is a list containing potential crush [x,y],
//6th field is a list containing static collision between hippo and hound when they stay at their nodes
//7th field is to store the potential nodes of colliding while driving

//pcn: potential crushing node, cd: colliding node, dc: drive colliding node
class node:
  def __init__(self,x,y,delta,pcn,cd,dc):
      self.x = x
      self.y    




typedef struct {
  double x;
  double y;
  double delta;
  double parent;
  double collision;
  double 

}

node init_node; //init node
int init_lock = 0;

static WbDeviceTag wheels[4];
static WbDeviceTag gps;
static WbDeviceTag compass;
static goto_struct goto_data;


static void base_set_wheel_velocity(WbDeviceTag t, double velocity) {
  wb_motor_set_position(t, INFINITY);
  wb_motor_set_velocity(t, velocity);
}

static void base_set_wheel_speeds_helper(double speeds[4]) {
  int i;
  for (i = 0; i < 4; i++)
    base_set_wheel_velocity(wheels[i], speeds[i]);
}

void base_init() {
  int i;
  char wheel_name[16];
  for (i = 0; i < 4; i++) {
    sprintf(wheel_name, "wheel%d", (i + 1));
    wheels[i] = wb_robot_get_device(wheel_name);
  }
}

void base_reset() {
  static double speeds[4] = {0.0, 0.0, 0.0, 0.0};
  base_set_wheel_speeds_helper(speeds);
}

void base_forwards() {
  static double speeds[4] = {SPEED, SPEED, SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_backwards() {
  static double speeds[4] = {-SPEED, -SPEED, -SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_turn_left() {
  static double speeds[4] = {-SPEED, SPEED, -SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_turn_right() {
  static double speeds[4] = {SPEED, -SPEED, SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_strafe_left() {
  static double speeds[4] = {SPEED, -SPEED, -SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_strafe_right() {
  static double speeds[4] = {-SPEED, SPEED, SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_80() {
 // get sensors
  const double *gps_raw_values = wb_gps_get_values(gps);
  const double *compass_raw_values = wb_compass_get_values(compass);

  // compute 2d vectors
  Vector2 v_gps = {gps_raw_values[0], gps_raw_values[2]};
  Vector2 v_front = {compass_raw_values[0], compass_raw_values[1]};
  Vector2 v_right = {-v_front.v, v_front.u};
  Vector2 v_north = {1.0, 0.0};

  // compute distance
  Vector2 v_dir;
  vector2_minus(&v_dir, &goto_data.v_target, &v_gps);
  double distance = vector2_norm(&v_dir);

  double speeds[4] = {0.0, 0.0, 0.0, 0.0};
  

  // -> second stimulus: u coord of the relative target vector
  speeds[0] += 0.1956* SPEED* 0.7071; //0.8944
  speeds[1] += 0.1956* SPEED* 0.7071;
  speeds[2] += 0.1956* SPEED* 0.7071;
  speeds[3] += 0.1956* SPEED* 0.7071;

  // -> third stimulus: v coord of the relative target vector
  speeds[0] -= 0.135* SPEED * 0.7071; //0.4472
  speeds[1] += 0.135* SPEED * 0.7071;
  speeds[2] += 0.135* SPEED * 0.7071;
  speeds[3] -=  0.135* SPEED * 0.7071;
  
  base_set_wheel_speeds_helper(speeds);
  
  if (distance < DISTANCE_TOLERANCE)
      goto_data.reached = true;
}


//This function is called everytime a newtarget is set for hippo to run into
void base_goto_init(double time_step) {
  gps = wb_robot_get_device("gps");
  compass = wb_robot_get_device("compass");
  if (gps)
    wb_gps_enable(gps, time_step);
  if (compass)
    wb_compass_enable(compass, time_step);
  if (!gps || !compass)
    fprintf(stderr, "cannot use goto feature without GPS and Compass");

  goto_data.v_target.u = 0.0;
  goto_data.v_target.v = 0.0;
  goto_data.alpha = 0.0;
  goto_data.time = 0.0;
  goto_data.reached = false;
}

void base_goto_set_target(double x, double z, double alpha) {
  if (!gps || !compass)
    fprintf(stderr, "base_goto_set_target: cannot use goto feature without GPS and Compass");

  goto_data.v_target.u = x;
  goto_data.v_target.v = z;
  goto_data.alpha = alpha;
  //goto_data.time_to_go = x/
  goto_data.reached = false;
}

void base_goto_run() {
  if (!gps || !compass)
    fprintf(stderr, "base_goto_set_target: cannot use goto feature without GPS and Compass");

  // get sensors
  const double *gps_raw_values = wb_gps_get_values(gps);
  const double *compass_raw_values = wb_compass_get_values(compass);

  // compute 2d vectors
  Vector2 v_gps = {gps_raw_values[0], gps_raw_values[2]};
  Vector2 v_front = {compass_raw_values[0], compass_raw_values[1]};
  Vector2 v_right = {-v_front.v, v_front.u};
  Vector2 v_north = {1.0, 0.0};

  // compute distance
  Vector2 v_dir;
  vector2_minus(&v_dir, &goto_data.v_target, &v_gps);
  double distance = vector2_norm(&v_dir);

  // compute absolute angle & delta with the delta with the target angle
  double theta = vector2_angle(&v_front, &v_north);
  double delta_angle = theta - goto_data.alpha;

  // compute the direction vector relatively to the robot coordinates
  // using an a matrix of homogenous coordinates
  Matrix33 transform;
  matrix33_set_identity(&transform);
  transform.a.u = v_front.u;
  transform.a.v = v_right.u;
  transform.b.u = v_front.v;
  transform.b.v = v_right.v;
  transform.c.u = -v_front.u * v_gps.u - v_front.v * v_gps.v;
  transform.c.v = -v_right.u * v_gps.u - v_right.v * v_gps.v;
  Vector3 v_target_tmp = {goto_data.v_target.u, goto_data.v_target.v, 1.0};
  Vector3 v_target_rel;
  matrix33_mult_vector3(&v_target_rel, &transform, &v_target_tmp);

  // compute the speeds
  double speeds[4] = {0.0, 0.0, 0.0, 0.0};
  
  // -> first stimulus: delta_angle
  speeds[0] = -delta_angle / M_PI * K1;
  speeds[1] = delta_angle / M_PI * K1;
  speeds[2] = -delta_angle / M_PI * K1;
  speeds[3] = delta_angle / M_PI * K1;

  // -> second stimulus: u coord of the relative target vector
  speeds[0] += v_target_rel.u * K2;
  speeds[1] += v_target_rel.u * K2;
  speeds[2] += v_target_rel.u * K2;
  speeds[3] += v_target_rel.u * K2;

  // -> third stimulus: v coord of the relative target vector
  speeds[0] += -v_target_rel.v * K3;
  speeds[1] += v_target_rel.v * K3;
  speeds[2] += v_target_rel.v * K3;
  speeds[3] += -v_target_rel.v * K3;

  // apply the speeds
  int i;
  for (i = 0; i < 4; i++) {
    speeds[i] /= (K1 + K2 + K2);  // number of stimuli (-1 <= speeds <= 1)
    //speeds[i] /= (K2 + K3);
    speeds[i] *= SPEED;           // map to speed (-SPEED <= speeds <= SPEED)

    // added an arbitrary factor increasing the convergence speed
    speeds[i] *= 200.0; //30
    speeds[i] = bound(speeds[i], -SPEED, SPEED);
  }
  printf("speed is %f",speeds[0]);
  base_set_wheel_speeds_helper(speeds);

  // check if the taget is reached
    if (distance < DISTANCE_TOLERANCE && delta_angle < ANGLE_TOLERANCE && delta_angle > -ANGLE_TOLERANCE)
      goto_data.reached = true;

}


bool base_goto_reached() {
  return goto_data.reached;
}


void base_goto_straight_to_direction(double init_x, double init_z, double x_t, double z_t) {

  double angle;
  double x, z;
  //x = x_t;
  //z = z_t;
  x = x_t - init_x;
  z = z_t - init_z;
  if (x == 0) {
    if(z >=0) {
      angle = -3.141/2;
      }
    else {
      angle = 3.141/2;
    }
  }
  else {
    angle = atan(z/x);
    if(x >= 0 && z >=0) {
      angle = angle * -1;    
    }
    if (x < 0 && z >= 0) {
      angle = -3.141 - angle;
    }
    if (x < 0 && z < 0) {
      angle = 3.141 - angle;
    }
    if (x > 0 && z < 0) {
      angle = angle * -1;
    }
  }
  
  base_goto_set_target(x_t,z_t,angle);
  base_goto_run();
  if (base_goto_reached()) {
    base_reset();
    lock = 1;
  }
}


void base_goto_rotate(double angle) {
  const double *gps_raw_values = wb_gps_get_values(gps);    
  base_goto_set_target(gps_raw_values[0], gps_raw_values[2],angle);
  base_goto_run();
  if (base_goto_reached()) {
    base_reset();
    lock = 0;
    init_lock = 0;
    node_num ++;
  }
}

void base_run(double x, double z, double alpha) {

if (lock == 0) {
  if (init_lock == 0) {
    const double *gps_raw_values = wb_gps_get_values(gps);
    init_node.x = gps_raw_values[0];
    init_node.z = gps_raw_values[2];
  }
  init_lock = 1;
  base_goto_straight_to_direction(init_node.x, init_node.z, x, z);
}
if (lock == 1) {
  base_goto_rotate(alpha);
}
}

//=================================================================

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}


static void display_helper_message() {
  printf("Control commands:\n");
  printf(" Arrows:       Move the robot\n");
  printf(" Page Up/Down: Rotate the robot\n");
  printf(" +/-:          (Un)grip\n");
  printf(" Shift + arrows:   Handle the arm\n");
  printf(" Space: Reset\n");
}

int main(int argc, char **argv) {
  
  wb_robot_init();

  base_init();
  //passive_wait(2.0);
  base_goto_init(TIME_STEP);
  if (argc > 1 && strcmp(argv[1], "demo") == 0)

  display_helper_message();

  int pc = 0;
  wb_keyboard_enable(TIME_STEP);
  
  //declare many nodes for hippo to follow
  node path[3];
  path[0].x = 1; path[1].x = -1; path[2].x = 2;
  path[0].z = 0; path[1].z = 2; path[2].z = -1;
  path[0].alpha = 0.5; path[1].alpha = 2.3; path[2].alpha = -0.5;
  

  //while loop to receive trajectory from hawk
  
  

  //=========end while loop===============
  while (true) {
    step();
    // assume the rebot is at position (0,0)
    // actuator and sensors changes evalues every 32 msec
    
    
    //if the reached node that has potential crushing nodes, send message asking where houd is at
    //hound responds with 3 possibilities of state: in the potential crushing node, not in potential crushing node
    // or travelling on the colliding path, not travelling on the colliding path.
    //if the other robot is in potential crushing node or in colliding path, pause. let the robot move. wait for free colliding
    
    
    
    
     
    base_goto_set_target(1,-1,0);
    //base_goto_run();
    //if (base_goto_reached()) {
      //base_reset();
    //}
    
   base_80();
    if (base_goto_reached()) {
       base_reset();
    }
    //base_strafe_right();
    base_forwards();    
    /*
    if (node_num < 3) {
      base_run(path[node_num].x,path[node_num].z,path[node_num].alpha);
    }
    */
    //base_goto_rotate(1.57); 
    /*
    //base_turn_left();
    int c = wb_keyboard_get_key();
    if ((c >= 0) && c != pc) {
      switch (c) {
        case WB_KEYBOARD_UP:
          printf("Go forwards\n");
          base_forwards();
          break;
        case WB_KEYBOARD_DOWN:
          printf("Go backwards\n");
          base_backwards();
          break;
        case WB_KEYBOARD_LEFT:
          printf("Strafe left\n");
          base_strafe_left();
          break;
        case WB_KEYBOARD_RIGHT:
          printf("Strafe right\n");
          base_strafe_right();
          break;
        case WB_KEYBOARD_PAGEUP:
          printf("Turn left\n");
          base_turn_left();
          break;
        case WB_KEYBOARD_PAGEDOWN:
          printf("Turn right\n");
          base_turn_right();
          break;
        case WB_KEYBOARD_END:
        case ' ':
          printf("Reset\n");
          base_reset();
          break;
        case '+':
        case 388:
        case 65585:
          printf("Grip\n");

          break;
        case '-':
        case 390:
          printf("Ungrip\n");
          break;
        case 332:
        case WB_KEYBOARD_UP | WB_KEYBOARD_SHIFT:
          printf("Increase arm height\n");
          break;
        case 326:
        case WB_KEYBOARD_DOWN | WB_KEYBOARD_SHIFT:
          printf("Decrease arm height\n");
          break;
        case 330:
        case WB_KEYBOARD_RIGHT | WB_KEYBOARD_SHIFT:
          printf("Increase arm orientation\n");
          break;
        case 328:
        case WB_KEYBOARD_LEFT | WB_KEYBOARD_SHIFT:
          printf("Decrease arm orientation\n");
          break;
        default:
          fprintf(stderr, "Wrong keyboard input\n");
          break;
      }
    }
    pc = c;
    */
  }

  wb_robot_cleanup();

  return 0;
}
