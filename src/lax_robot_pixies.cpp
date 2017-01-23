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

#define BLOCK_BUFFER_SIZE    25
#define MIN_BALL_SIZE        10
#define NUM_PIXIES						2
#define FOCAL_LENGTH				2.8
#define CAMERA_SEPARATION		610.0

using std::vector;

// Pixy Block buffer // 

static bool run_flag = true;

void handle_SIGINT(int unused)
{
	// On CTRL+C - abort! //

	run_flag = false;
}

// using washington.edu algorithm
int print_xyz(struct Block* blocks)
{
	double x,y,z;
	double x1, x2;
	double y1, y2;

	struct Block left = blocks[0];
	struct Block right = blocks[1];


	z = (FOCAL_LENGTH * CAMERA_SEPARATION) / (left.x - right.x);

	x1 = (left.x * z) / FOCAL_LENGTH;
	x2 = ((right.x * z) / FOCAL_LENGTH) + CAMERA_SEPARATION;
	x = (x1 + x2) / 2;

	y1 = (-(left.y-100) * z ) / FOCAL_LENGTH;
	y2 = (-(right.y-100) * z) / FOCAL_LENGTH;
	y = (y1 + y2) / 2;

    x += .5 * CAMERA_SEPARATION;
    x = x * -1;
    y = y * -1;
    z = z * -1;
    // note: z is not scaled correctly.


	fprintf(stderr, "3D Point Coordinate: X: %.2f, Y: %.2f, Z: %.2f.\n", x, y, z); 
}

int main(int argc, char * argv[])
{
	// Catch CTRL+C (SIGINT) signals //
	signal(SIGINT, handle_SIGINT);

	// confirm correct number of pixies available
	if (PixyHandle::num_pixies_attached() != NUM_PIXIES) 
	{
		fprintf(stderr, "Invalid number of pixies connected. Please connect 2 pixies.\n");
		return EXIT_FAILURE;
	}

	fprintf(stderr, "Hello Pixies:\n libpixyusb Version: %s\n", __LIBPIXY_VERSION__);

	// Initialize all pixy cameras
	vector<int> pixy_init_status(NUM_PIXIES);
	vector<PixyHandle> pixy_handles(NUM_PIXIES);
	for (int i = 0; i < NUM_PIXIES; i++) 
	{
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

			if (return_value) 
			{
				// Error //
				fprintf(stderr, "Failed to retrieve Pixy firmware version. ");
				pixy_handles[i].error(return_value);
				return return_value;
			} 
			else 
			{
				// Success //
				fprintf(stderr, " Pixy Firmware Version: %d.%d.%d\n", major, minor, build);
			}
		}
	}

	int      frame = 0; // currently unused
	int      index;
	char     buf[128];

	// blocks that will be passed to triangulation code
	struct Block *target_blocks = new struct Block[NUM_PIXIES];

	vector<int> blocks_copied(NUM_PIXIES);
	struct Block **blocks = new struct Block*[NUM_PIXIES];
	for (int i = 0 ; i < NUM_PIXIES; i++) 
	{
		blocks[i] = new struct Block[BLOCK_BUFFER_SIZE];
	}

	fprintf(stderr, "Detecting blocks...\n");
	while (run_flag)
	{
		// Wait for new blocks to be available (object in FOV of both) //
		for (int i = 0; i < NUM_PIXIES; i++) {
			while(!pixy_handles[i].blocks_are_new() && run_flag);
		}

		if (run_flag)
		{
            bool valid_signal = true;
			for (int i = 0; i < NUM_PIXIES; i++) 
			{
				// Get blocks from Pixy //
				blocks_copied[i] = pixy_handles[i].get_blocks(BLOCK_BUFFER_SIZE, &(blocks[i][0]));

				if (blocks_copied[i] < 0) 
				{
					// Error: pixy_get_blocks //
					fprintf(stderr, "pixy_get_blocks(): ");
					pixy_handles[i].error(blocks_copied[i]);
				}

				//Check for maximum area block
				int max_size = 0;
				int max_index = 0;
				for (int j = 0; j != blocks_copied[i]; ++j) 
				{
					int curr_size = blocks[i][j].width * blocks[i][j].height;
					if (curr_size > max_size) 
					{
						max_size = curr_size;
						max_index = j;
					}
				}
                if (max_size < MIN_BALL_SIZE)
                    valid_signal = false;

				target_blocks[i] = blocks[i][max_index];
			}
            if (!valid_signal)
                continue;

			print_xyz(target_blocks);

			frame++;
		}
	}

	for (int i = 0; i < NUM_PIXIES; i++) 
	{
		pixy_handles[i].close();
		delete[] blocks[i];
	}

	delete[] blocks;

	return EXIT_SUCCESS;
}
