/*
 * File:          hound_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
 
 
#include <stdio.h>
#include <string.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>

/*
 * You may want to add macros here.
 */
 
 
#define MAX_SPEED 10

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
 
 
void process (const char * data)
{
  if (strncmp(data, "Hound", 5) == 0)
  {
    const char *message = &data[7];
    printf("Received \"%s\"\n\n", message);
  }
  else
  {
    printf("Message not for me, ignored \n");
  }
}
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  int timestep = (int)wb_robot_get_basic_time_step();
  wb_keyboard_enable(timestep);

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  WbDeviceTag emitter = wb_robot_get_device("emitter");
  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, timestep);
  
  
  
  WbDeviceTag wheels[3];
  char wheels_names[3][8] = {"wheel1", "wheel2", "boop"};
  for (int i = 0; i < 3; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    
    if (i == 2)
    {
    wb_motor_set_position(wheels[i], 0);
    continue;
    }
    
    wb_motor_set_position(wheels[i], INFINITY);
    wb_motor_set_velocity(wheels[i], 0);
  }
 
 
 while (wb_robot_step(timestep) != -1) {
    if (wb_robot_get_time() > 1.0)
      break;
  }
 
 
 double push = 0;
  
    // Display manual control message.
  //printf("You can control the hound with your computer keyboard:\n");
  //printf("- 'up': move forward.\n");
  //printf("- 'down': move backward.\n");
  //printf("- 'right': turn right.\n");
  //printf("- 'left': turn left.\n");
  //printf("- 'shift + down': stop.\n");
  //printf("- 'shift + right': spin right.\n");
  //printf("- 'shift + left': spin left.\n");

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(timestep) != -1) {
  
    double left_speed = 0;
    double right_speed = 0;
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
     
     if (wb_receiver_get_queue_length(receiver) > 0)
     {
       const char *buffer = wb_receiver_get_data(receiver);
       process(buffer);
       wb_receiver_next_packet(receiver);
     }

    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
     
    int key = wb_keyboard_get_key();
    while (key > 0) {
      switch (key) {
        case WB_KEYBOARD_UP:
          left_speed = MAX_SPEED;
          right_speed = MAX_SPEED;
          break;
        case WB_KEYBOARD_DOWN:
          left_speed = -MAX_SPEED;
          right_speed = -MAX_SPEED;
          break;
        case WB_KEYBOARD_RIGHT:
          left_speed = MAX_SPEED;
          right_speed = 0*MAX_SPEED;
          break;
        case WB_KEYBOARD_LEFT:
          left_speed = 0*MAX_SPEED;
          right_speed = MAX_SPEED;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_RIGHT):
          left_speed = MAX_SPEED;
          right_speed = -MAX_SPEED;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_LEFT):
          left_speed = -MAX_SPEED;
          right_speed = MAX_SPEED;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_DOWN):
          push = .84;
          break;
        case (WB_KEYBOARD_SHIFT + WB_KEYBOARD_UP):
          push = 0;
          break;
      }
      key = wb_keyboard_get_key();
    } 
     
    wb_motor_set_velocity(wheels[0], left_speed);
    wb_motor_set_velocity(wheels[1], right_speed);
    //wb_motor_set_velocity(wheels[2], right_speed);
    //wb_motor_set_velocity(wheels[3], right_speed);
    wb_motor_set_position(wheels[2], push);
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}