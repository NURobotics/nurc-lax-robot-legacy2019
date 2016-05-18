#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <signal.h>

#include <string>

#include <lax_robot.h>

using std::cin;
using std::cout;
using std::endl;
using std::string;

const string kMotorControllerPort = "/dev/ttyACM0";
const int kMotorControllerBaudRate = 9600;

bool stop_flag = false;

void sigint_handler(int signo)
{
  stop_flag = true;
}

int main(int argc, char **argv)
{
  if (signal(SIGINT, sigint_handler) == SIG_ERR) {
    return EXIT_FAILURE;
  }

  cout << "==================" << endl;
  cout << "Hit Ctrl-C to exit" << endl;
  cout << "==================" << endl;
  cout << endl;

  LaxRobot lax_robot(kMotorControllerPort, kMotorControllerBaudRate);
  lax_robot.start();
  while (!stop_flag);
  lax_robot.stop();

  return EXIT_SUCCESS;
}
