//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>

#include <vector>
#include <cmath>

#include "pixyhandle.hpp"
#include "lax_robot_pixies_functions.hpp"

#define BLOCK_BUFFER_SIZE    25
#define MIN_BALL_SIZE        10

using std::vector;

// Pixy Block buffer // 

static bool run_flag = true;

void handle_SIGINT(int unused)
{
  // On CTRL+C - abort! //

  run_flag = false;
}

int main(int argc, char * argv[])
{
  // Catch CTRL+C (SIGINT) signals //
  signal(SIGINT, handle_SIGINT);

  int num_pixies = PixyHandle::num_pixies_attached();
  if (num_pixies != 2) {
    fprintf(stderr, "Invalid number of pixies connected. Please connect 2 pixies.\n");
    return EXIT_FAILURE;
  }

  fprintf(stderr, "Hello Pixies:\n libpixyusb Version: %s\n", __LIBPIXY_VERSION__);

  vector<int>         pixy_init_status(num_pixies);
  vector<PixyHandle>  pixy_handles(num_pixies);
  for (int i = 0; i < num_pixies; i++) {
    // Connect to Pixy //
    pixy_init_status[i] = pixy_handles[i].init();

    // Was there an error initializing pixy? //
    if(pixy_init_status[i] != 0)
    {
      // Error initializing Pixy //
      fprintf(stderr, "pixy_init(): ");
      pixy_handles[i].error(pixy_init_status[i]);

      return pixy_init_status[i];
    }

    // Request Pixy firmware version //
    {
      uint16_t major;
      uint16_t minor;
      uint16_t build;
      int      return_value;

      return_value = pixy_handles[i].get_firmware_version(&major, &minor, &build);

      if (return_value) {
        // Error //
        fprintf(stderr, "Failed to retrieve Pixy firmware version. ");
        pixy_handles[i].error(return_value);

        return return_value;
      } else {
        // Success //
        fprintf(stderr, " Pixy Firmware Version: %d.%d.%d\n", major, minor, build);
      }
    }
  }

  int      frame = 0;
  int      index;
  char     buf[128];
  //int prev_frame_x = 0;
  //int prev_frame_y = 0;
  double x_fov = 75.0;
  double y_fov = 47.0;
  double max_xpos = 320.0;
  double max_ypos = 240.0;
  int pos_array[2][2];
  double angle_array[2][2];
  int size_array[2];
  double lines[2][3];
  double cam1_x = -12.0;
  double cam1_y = 17.0;
  double cam1_z = 0;
  double cam2_x = 12.0;
  double cam2_y = 17.0;
  double cam2_z = 0;

  double line_matrix[3][3];
  double inner_prod1[3];
  double inner_prod2[3];
  double solve_matrix[2][3];
  double t;
  double s;
  double point1[3];
  double point2[3];
  double midpoint[3];


  vector<int>      blocks_copied(num_pixies);
  struct Block **blocks = new struct Block*[num_pixies];
  for (int i = 0 ; i < num_pixies; i++) {
    blocks[i] = new struct Block[BLOCK_BUFFER_SIZE];
  }

  fprintf(stderr, "Detecting blocks...\n");
  while(run_flag)
  {
    // fprintf(stderr, "frame %d:\n", frame);
    // Wait for new blocks to be available //
    for (int i = 0; i < num_pixies; i++) {
      while(!pixy_handles[i].blocks_are_new() && run_flag); 
    }

    if (run_flag) {
      for (int i = 0; i < num_pixies; i++) {
        // Get blocks from Pixy //
        blocks_copied[i] = pixy_handles[i].get_blocks(BLOCK_BUFFER_SIZE, &(blocks[i][0]));

        //fprintf(stderr, " camera %d:\n", i);

        if(blocks_copied[i] < 0) {
          // Error: pixy_get_blocks //
          fprintf(stderr, "pixy_get_blocks(): ");
          pixy_handles[i].error(blocks_copied[i]);
        }

        //Check for maximum size block
        int max_size = 0;
        int max_index = 0;
        for (index = 0; index != blocks_copied[i]; ++index) {
          int curr_size = blocks[i][index].width * blocks[i][index].height;
          if (curr_size > max_size) {
            max_size = curr_size;
            max_index = index;
          }
        }
        pos_array[i][0] = blocks[i][max_index].x;
        pos_array[i][1] = blocks[i][max_index].y;
        size_array[i] = max_size;
        //printf("Maximum size block has index: %d, size: %d\n", max_index, max_size);
        //printf("Maximum size block has size %d\n", max_size);


        //Check for closest block to the most relevant block of the last frame
        // int min_diff = 10000;
        // int min_index = 0;
        // for (index = 0; index != blocks_copied[i]; ++index) {
        //   int x_diff = abs(blocks[i][index].x - prev_frame_x);
        //   int y_diff = abs(blocks[i][index].y - prev_frame_y);
        //   if (x_diff + y_diff < min_diff) {
        //     min_diff = x_diff + y_diff;
        //     min_index = index;
        //   }
        // }
        // printf("Closest block has index %d\n", min_index);
        // printf("Closest block has position difference %d\n", min_diff);

        // prev_frame_x = blocks[i][min_index].x;
        // prev_frame_y = blocks[i][min_index].y;
      }

      //NEED TO CHECK TO MAKE SURE THAT MAX SIZE BLOCK DETECTED BY BOTH CAMERAS IS THE SAME BLOCK
      //Do the triangulation
      if (size_array[0] > MIN_BALL_SIZE && size_array[1] > MIN_BALL_SIZE)
      {
        CalculateAngles(x_fov, y_fov, max_xpos, max_ypos, pos_array, angle_array);

        CalculateLineVectors(angle_array, lines);

        CreateLineMatrix(line_matrix, lines, cam1_x, cam1_y, cam1_z, cam2_x, cam2_y, cam2_z);

        InnerProduct(line_matrix, lines[0], inner_prod1);
        InnerProduct(line_matrix, lines[1], inner_prod2);
        //printf("Inner Products: %f, %f, %f, %f, %f, %f.\n", inner_prod1[0],inner_prod1[1],inner_prod1[2],inner_prod2[0],inner_prod2[1],inner_prod2[2]);

        CreateSolveMatrix(solve_matrix, inner_prod1, inner_prod2);

        RowReduce(solve_matrix);
        t = solve_matrix[0][2];
        s = solve_matrix[1][2];
        //printf("Solved matrix values: %f, %f.\n",t,s);

        FindPoints(cam1_x, cam1_y, cam1_z, cam2_x, cam2_y, cam2_z, lines, point1, point2, midpoint, t, s);

        // if (frame % 100 == 0)
        fprintf(stderr, "3D Point Coordinate: X: %f, Y: %f, Z: %f.\n", midpoint[0],midpoint[1],midpoint[2]);
      } 
      else {
        //printf("No matching blocks found.\n");
      }
    }

    frame++;
  }

  for (int i = 0; i < num_pixies; i++) {
    pixy_handles[i].close();
  }

  for (int i = 0; i < num_pixies; i++) {
    delete[] blocks[i];
  }
  delete[] blocks;

  return EXIT_SUCCESS;
}
