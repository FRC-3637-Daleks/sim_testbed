// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

#include <iostream>

void Robot::RobotInit()
{
  std::cout << "Robot Init" << std::endl;
  frc2::CommandScheduler::GetInstance().OnCommandInitialize([](const frc2::Command &c)
  {
    std::cout << "Running " << c.GetName() << std::endl;
  });
  frc2::CommandScheduler::GetInstance().OnCommandFinish([](const frc2::Command &c)
  {
    std::cout << "Finished " << c.GetName() << std::endl;
  });
  frc2::CommandScheduler::GetInstance().OnCommandInterrupt([](const frc2::Command &c)
  {
    std::cout << "Interrupting " << c.GetName() << std::endl;
  });
  
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::SimulationPeriodic()
{
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  std::cout << "Teleop Init" << std::endl;
  m_container.drive_base_.Reconfigure();
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  std::cout << "Test Init" << std::endl;
}

void Robot::TestPeriodic()
{
}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
