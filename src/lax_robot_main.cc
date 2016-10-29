#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <signal.h>

#include <string>

// #include <boost/program_options.hpp>

#include <lax_robot.h>

using std::cin;
using std::cout;
using std::cerr;
using std::endl;
using std::string;

// namespace po = boost::program_options;

bool stop_flag = false;

void sigint_handler(int signo)
{
  stop_flag = true;
}
 
int main(int argc, char **argv)
{
  string motor_port = "/dev/ttyO2";
  int motor_baud = 115200;
  
  /*try {
    po::options_description desc("Options");
    desc.add_options()
      ("help,h", "The motor controller serial port and baud rate must be supplied")
      ("motor_serial_port,p", po::value<string>(), "Motor controller serial port filename")
      ("motor_serial_baud_rate,b", po::value<int>(), "Motor controller serial baud rate");
     
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help")) {
      cout << desc << endl;
      return EXIT_SUCCESS;
    }
    
    if (vm.count("motor_serial_port") &&
        vm.count("motor_serial_baud_rate")) {
      motor_port = vm["motor_serial_port"].as<string>();
      motor_baud = vm["motor_serial_baud_rate"].as<int>();
    } else {
      cout << desc << endl;
      return EXIT_FAILURE;
    }
  } catch(po::error& e) {
    cerr << "Error parsing command line: " << e.what() << endl;
    return EXIT_FAILURE;
  } catch(std::exception& e) {
    cerr << "Could not handle exception: " << e.what() << endl;
    return EXIT_FAILURE;
  }*/
  
  if (signal(SIGINT, sigint_handler) == SIG_ERR) {
    return EXIT_FAILURE;
  }

  cout << "==================" << endl;
  cout << "Hit Ctrl-C to exit" << endl;
  cout << "==================" << endl;
  cout << endl;

  LaxRobot lax_robot(motor_port, motor_baud);
  if (!lax_robot.start()) {
    cerr << "[Lax Robot] Failed to start up the lax robot" << endl;
    return EXIT_FAILURE;
  }
  
  while (lax_robot.ok() && !stop_flag);
  lax_robot.stop();

  return EXIT_SUCCESS;
}
