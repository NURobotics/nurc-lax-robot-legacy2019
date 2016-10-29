#include "triangulation.h"

#include "math.h"
#include "assert.h"
#include "stdio.h"
#include "stdlib.h"

#define TEST_TOLERANCE (1e-4)

bool TestRowReduce()
{
  printf("---------------\n");
  printf("Test Row Reduce\n");
  printf("---------------\n");
  double test_matrix[2][2][3] = {
    {
      {2, 0, 0},
      {0, 10, 0},
    },
    {
      {1, 10, 8},
      {9, 7, 8},
    },
  };

  double test_solution[2][2][3] = {
    {
      {1, 0, 0},
      {0, 1, 0},
    },
    {
      {1, 0, 0.2891566}, 
      {0, 1, 0.77110843},
    },
  };

  for (int i = 0; i < 2; i++) {
    RowReduce(test_matrix[i]);
  }

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      for (int k = 0; k < 3; k++) {
        if (fabs(test_matrix[i][j][k] - test_solution[i][j][k]) > TEST_TOLERANCE) {
          return false;
        }
        printf("%f ", test_matrix[i][j][k]);
      }
      printf("\n");
    }
    printf("\n");
  }
  
  printf("---------------------\n");
  printf("Golly Gee Good Job!!!\n");
  printf("---------------------\n\n");
  return true;
}

bool TestInnerProduct()
{
  printf("------------------\n");
  printf("Test Inner Product\n");
  printf("------------------\n");
  double test_matrix[2][3][3] = {
    {
      {1, 0, 0},
      {0, 1, 0},
      {0, 0, 1},
    },
    {
      {4, 8, 1},
      {7, 1, 1},
      {2, 3, 9},
    },
  };

  double test_vector[2][3] = {
    {1, 2, 3},
    {1, 2, 3},
  };

  double test_solution[2][3] = {
    {1, 2, 3}, 
    {23, 12, 35},
  };

  double test_output[2][3];

  for (int i = 0; i < 2; i++) {
    InnerProduct(test_matrix[i], test_vector[i], test_output[i]);
  }

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 3; j++) {
      if (fabs(test_solution[i][j] - test_output[i][j]) > TEST_TOLERANCE) {
        return false;
      }
      printf("%f ", test_output[i][j]);
    }
    printf("\n");
  }

  printf("---------------------\n");
  printf("Golly Gee Good Job!!!\n");
  printf("---------------------\n\n");
  return true;
}

bool TestCalculateAngles()
{
  printf("---------------------\n");
  printf("Test Calculate Angles\n");
  printf("---------------------\n");
  const double xfov = 75;
  const double yfov = 47;
  const double max_xpos = 320;
  const double max_ypos = 240;

  int test_points[5][2][2] = {
    {
      {0, 0},
      {0, 0},
    },
    {
      {0, max_ypos},
      {0, max_ypos},
    },
    {
      {max_xpos, max_ypos},
      {max_xpos, max_ypos},
    },
    {
      {max_xpos, 0},
      {max_xpos, 0},
    },
    {
      {max_xpos / 2, max_ypos / 2},
      {max_xpos / 2, max_ypos / 2},
    },
  };

  double test_solution[5][2] = {
    {-xfov / 2, yfov / 2},
    {-xfov / 2, -yfov / 2},
    {xfov / 2, -yfov / 2},
    {xfov / 2, yfov / 2},
    {0, 0},
  };

  double test_output[5][2][2];
  for (int i = 0; i < 5; i++) {
    CalculateAngles(xfov, yfov, max_xpos, max_ypos, test_points[i], test_output[i]);
  }

  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 2; j++) {
      printf("%f ", test_output[i][0][j]);
      for (int k = 0; k < 2; k++) {
        if (fabs(test_solution[i][j] - test_output[i][k][j]) > TEST_TOLERANCE) {
          return false;
        }
      }
    }
    printf("\n");
  }

  printf("---------------------\n");
  printf("Golly Gee Good Job!!!\n");
  printf("---------------------\n\n");
  return true;
}

bool TestCalculateLineVectors()
{
  printf("---------------------------\n");
  printf("Test Calculate Line Vectors\n");
  printf("---------------------------\n");

  double test_angles[9][2][2] = {
    {
      {0, 0},
      {0, 0},
    },
    {
      {-45, 0},
      {-45, 0},
    },
    {
      {45, 0},
      {45, 0},
    },
    {
      {-45, 45},
      {-45, 45},
    },
    {
      {-45, -45},
      {-45, -45},
    },
    {
      {45, 45},
      {45, 45},
    },
    {
      {45, -45},
      {45, -45},
    },
    {
      {0, -45},
      {0, -45},
    },
    {
      {0, 45},
      {0, 45},
    },
  };

  double test_solution[9][3] = {
    {0, 0, 1},
    {-0.70710678, 0, 0.70710678,},
    {0.70710678, 0, 0.70710678,},
    {-0.57735027, 0.57735027, 0.57735027,},
    {-0.57735027, -0.57735027, 0.57735027,},
    {0.57735027, 0.57735027, 0.57735027,},
    {0.57735027, -0.57735027, 0.57735027,},
    {0, -0.70710678, 0.70710678,},
    {0, 0.70710678, 0.70710678,},
  };
  
  double test_output[9][2][3];
  for (int i = 0; i < 9; i++) {
    CalculateLineVectors(test_angles[i], test_output[i]);
  }

  for (int i = 0; i < 9; i++) {
    double norm = sqrt(test_output[i][0][0] * test_output[i][0][0] +
                       test_output[i][0][1] * test_output[i][0][1] +
                       test_output[i][0][2] * test_output[i][0][2]);
    for (int j = 0; j < 3; j++) {
      printf("%f ", test_output[i][0][j]);
      for (int k = 0; k < 2; k++) {
        if (fabs(test_output[i][k][j] / norm - test_solution[i][j]) > TEST_TOLERANCE) {
          return false;
        }
      }
    }
    printf("\n");
  }
  
  printf("---------------------\n");
  printf("Golly Gee Good Job!!!\n");
  printf("---------------------\n\n");
  return true;
}

bool TestCreateLineMatrix()
{
  printf("-----------------------\n");
  printf("Test Create Line Matrix\n");
  printf("-----------------------\n");

  printf("TODO\n");

  printf("---------------------\n");
  printf("Golly Gee Good Job!!!\n");
  printf("---------------------\n\n");
  return true;
}

bool TestCreateSolveMatrix()
{
  printf("------------------------\n");
  printf("Test Create Solve Matrix\n");
  printf("------------------------\n");

  printf("TODO\n");

  printf("---------------------\n");
  printf("Golly Gee Good Job!!!\n");
  printf("---------------------\n\n");
  return true;
}

bool TestFindPoints()
{
  printf("------------------------\n");
  printf("Test Create Solve Matrix\n");
  printf("------------------------\n");

  printf("TODO\n");

  printf("---------------------\n");
  printf("Golly Gee Good Job!!!\n");
  printf("---------------------\n\n");
  return true;
}

int main (int argc, char **argv)
{
  assert(TestRowReduce());
  assert(TestInnerProduct());
  assert(TestCalculateAngles());
  assert(TestCalculateLineVectors());
  assert(TestCreateLineMatrix());
  assert(TestFindPoints());
}
