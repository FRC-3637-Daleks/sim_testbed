// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include <frc/Joystick.h>

#include "DriveBase.h"

class RobotContainer {
public:
    RobotContainer();
    frc2::CommandPtr GetAutonomousCommand();

private:
    void ConfigureBindings();

public:
    DriveBase drive_base_;
    frc::Joystick joy_;
};
