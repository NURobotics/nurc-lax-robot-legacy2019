#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>

#include <vector>
#include <cmath>

#include "pixyhandle.hpp"
#include "triangulation.h"

void RowReduce(double A[2][3])
{
    const int nrows = 2; // number of rows
    const int ncols = 3; // number of columns

    int lead = 0; 

    while (lead < nrows) {
        double d, m;

        for (int r = 0; r < nrows; r++) { // for each row ...
            /* calculate divisor and multiplier */
            d = A[lead][lead];
            m = A[r][lead] / A[lead][lead];

            for (int c = 0; c < ncols; c++) { // for each column ...
                if (r == lead)
                    A[r][c] /= d;               // make pivot = 1
                else
                    A[r][c] -= A[lead][c] * m;  // make other = 0
            }
        }

        lead++;
    }
}

void InnerProduct(double A[3][3], double x[3], double b[3]) 
{
  	for (int i = 0; i < 3; i++) {
    	double sum = 0;
    	for (int j = 0; j < 3; j++) {
      		sum = sum + (A[i][j] * x[j]);
    	}
    	b[i] = sum;
  	}
}

void CalculateAngles(double x_fov, double y_fov, double max_xpos, double max_ypos, int pos_array[2][2], double angle_array[2][2])
{
  	angle_array[0][0] = ((pos_array[0][0]-(max_xpos/2))/max_xpos)*x_fov;
  	angle_array[0][1] = (((max_ypos/2)-pos_array[0][1])/max_ypos)*y_fov;
  	angle_array[1][0] = ((pos_array[1][0]-(max_xpos/2))/max_xpos)*x_fov;
  	angle_array[1][1] = (((max_ypos/2)-pos_array[1][1])/max_ypos)*y_fov;
}

void CalculateLineVectors(double angle_array[2][2], double lines[2][3])
{
	for (int i = 0; i < 2; i++)
	{
      lines[i][0] = tan((angle_array[i][0] * M_PI) / 180.0);
    	lines[i][1] = tan((angle_array[i][1] * M_PI) / 180.0);
    	lines[i][2] = 1;
  }
}

void CreateLineMatrix(double line_matrix[3][3], double lines[2][3], double cam1_x, double cam1_y, double cam1_z, double cam2_x, double cam2_y, double cam2_z)
{
	for (int i = 0; i < 3; i++) {
        line_matrix[0][i] = -lines[0][i];
    }
    for (int i = 0; i < 3; i++) {
        line_matrix[1][i] = lines [1][i];
    }

    line_matrix[2][0] = cam2_x - cam1_x;
    line_matrix[2][1] = cam2_y - cam1_y;
    line_matrix[2][2] = cam2_z - cam1_z;
}

void CreateSolveMatrix(double solve_matrix[3][3], double inner_prod1[3], double inner_prod2[3])
{
	for (int i = 0; i < 3; i++) {
    	solve_matrix[0][i] = inner_prod1[i];
    }
    for (int i = 0; i < 3; i++) {
        solve_matrix[1][i] = inner_prod2[i];
    }
}

void FindPoints(double cam1_x, double cam1_y, double cam1_z, double cam2_x, double cam2_y, double cam2_z, double lines[2][3], double* point1, double* point2, double* midpoint, double t, double s)
{
    point1[0] = cam1_x + (lines[0][0] * t);
    point1[1] = cam1_y + (lines[0][1] * t);
    point1[2] = cam1_z + (lines[0][2] * t);


    point2[0] = cam2_x + (lines[1][0] * s);
    point2[1] = cam2_y + (lines[1][1] * s);
    point2[2] = cam2_z + (lines[1][2] * s); 

    for (int i = 0; i < 3; i++) {
        midpoint[i] = (point1[i] + point2[i]) / 2;
    }

}

