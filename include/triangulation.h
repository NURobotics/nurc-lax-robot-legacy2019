#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>

#include <vector>
#include <cmath>

#include "pixyhandle.hpp"

void RowReduce(double A[2][3]);

void InnerProduct(double A[3][3], double x[3], double b[3]);

void CalculateAngles(double x_fov,
                     double y_fov,
                     double max_xpos,
                     double max_ypos,
                     int pos_array[2][2],
                     double angle_array[2][2]);

void CalculateLineVectors(double angle_array[2][2],
                          double lines[2][3]);

void CreateLineMatrix(double line_matrix[3][3],
                      double lines[2][3],
                      double cam1_x,
                      double cam1_y,
                      double cam1_z,
                      double cam2_x,
                      double cam2_y,
                      double cam2_z);

void CreateSolveMatrix(double solve_matrix[3][3],
                       double inner_prod1[3],
                       double inner_prod2[3]);

void FindPoints(double cam1_x,
                double cam1_y,
                double cam1_z,
                double cam2_x,
                double cam2_y,
                double cam2_z,
                double lines[2][3],
                double* point1,
                double* point2,
                double* midpoint,
                double t, double s);

#endif // TRIANGULATION_H
