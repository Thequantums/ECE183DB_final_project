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

#include <base.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/motor.h>

#define TIME_STEP 16

#define SPEED 4.0
#define DISTANCE_TOLERANCE 0.001
#define ANGLE_TOLERANCE 0.001

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

//static void display_helper_message() {
//  printf("Control commands:\n");
//  printf(" Arrows:       Move the robot\n");
//  printf(" Page Up/Down: Rotate the robot\n");
//  printf(" +/-:          (Un)grip\n");
//  printf(" Shift + arrows:   Handle the arm\n");
//  printf(" Space: Reset\n");
//}

void process (const char * data)
{
  if (strncmp(data, "Hippo", 5) == 0)
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
  wb_robot_init();

  WbDeviceTag emitter = wb_robot_get_device("emitter");
  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);

  base_init();

  //display_helper_message();

  int pc = 0;
  wb_keyboard_enable(TIME_STEP);

  while (true) {
    step();
    
    if (wb_receiver_get_queue_length(receiver) > 0)
     {
       const char *buffer = wb_receiver_get_data(receiver);
       process(buffer);
       wb_receiver_next_packet(receiver);
     }

    int c = wb_keyboard_get_key();
    if ((c >= 0) && c != pc) {
      switch (c) {
        case WB_KEYBOARD_UP:
          //printf("Go forwards\n");
          base_forwards();
          break;
        case WB_KEYBOARD_DOWN:
          //printf("Go backwards\n");
          base_backwards();
          break;
        case WB_KEYBOARD_LEFT:
          //printf("Strafe left\n");
          base_strafe_left();
          break;
        case WB_KEYBOARD_RIGHT:
          //printf("Strafe right\n");
          base_strafe_right();
          break;
        case WB_KEYBOARD_PAGEUP:
          //printf("Turn left\n");
          base_turn_left();
          break;
        case WB_KEYBOARD_PAGEDOWN:
          //printf("Turn right\n");
          base_turn_right();
          break;
        case WB_KEYBOARD_END:
        case ' ':
          base_reset();
          //printf("Reset\n");
          break;
        case '+':
        case 388:
        case 65585:
          //printf("Grip\n");
          break;
        case '-':
        case 390:
          //printf("Ungrip\n");
          break;
        default:
          //fprintf(stderr, "Wrong keyboard input\n");
          break;
      }
    }
    pc = c;
  }

  wb_robot_cleanup();

  return 0;
}
