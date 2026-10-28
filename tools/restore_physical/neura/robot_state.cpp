// robot_state [reset]: read-only LARA5 machine state (status bits, error code, trajectory scaling).
// With "reset" it opens a read-write client, calls resetErrors() and, if the motors are off,
// powerON(). Run it only with the controller stack stopped, and restart the stack afterwards.
#include <neura_cpp_api/robot.hpp>

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

int main(int argc, char** argv)
{
  const bool reset = argc > 1 && std::strcmp(argv[1], "reset") == 0;
  neura::Robot robot({192, 168, 2, 13}, reset ? neura::ClientMode::ReadWrite : neura::ClientMode::ReadOnly);
  uint64_t s = 0;
  for (int i = 0; i < 30 && !robot.getLastMachineState(s); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto bit = [&](int b) { return int((s >> b) & 1); };
  std::cout << "status_word 0x" << std::hex << s << std::dec << " power/b0=" << bit(0) << " no_motion/b1=" << bit(1)
            << " b2=" << bit(2) << " b3=" << bit(3) << " external/b4=" << bit(4) << " error/b5=" << bit(5)
            << " collision/b6=" << bit(6) << " real/b7=" << bit(7) << " interface=" << ((s >> 8) & 0xff)
            << " neura_cpp_writer/b16=" << bit(16) << std::endl;
  std::cout << "error_code " << robot.getErrorCode() << " scaling " << robot.getTrajectoryScalingFactor() << std::endl;
  std::vector<double> jl, cl;
  std::cout << "joint_limits(" << robot.getCurrentJointLimits(jl) << ")";
  for (double v : jl) std::cout << " " << v;
  std::cout << std::endl << "cartesian_limits(" << robot.getCurrentCartesianLimits(cl) << ")";
  for (double v : cl) std::cout << " " << v;
  std::cout << std::endl;
  if (reset)
  {
    std::cout << "resetErrors -> " << robot.resetErrors() << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    robot.getLastMachineState(s);
    if (!bit(0))
    {
      std::cout << "powerON -> " << robot.powerON() << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      robot.getLastMachineState(s);
    }
    std::cout << "status_word now 0x" << std::hex << s << std::dec << " power=" << bit(0) << " error=" << bit(5)
              << " collision=" << bit(6) << std::endl;
  }
  robot.shutdown();
  return 0;
}
