#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <hal/SimDevice.h>
#include <hal/simulation/SimDeviceData.h>

#include <ctre/Phoenix.h>
#include <AHRS.h>

#include <units/velocity.h>
#include <units/length.h>
#include <units/angle.h>

#include <frc/Joystick.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/controller/PIDController.h>

// Just using this for the smartdashboard UI element
class PIDFController: public frc::PIDController
{
public:
    PIDFController(double kF, double kP, double kI, double kD):
        frc::PIDController(kP, kI, kD), kF_(kF) {}
    
    double GetF() const {return kF_;}
    double SetF(double kF) {kF_ = kF;}
    void InitSendable(wpi::SendableBuilder& builder) override;

private:
    double kF_;
};

class DriveBase: public frc2::SubsystemBase
{
public:
    enum motor_index_t {LEFT_MAIN=0, LEFT_FOLLOWER, RIGHT_MAIN, RIGHT_FOLLOWER, N_MOTORS};

private:
    // devices
    WPI_TalonFX motors_[N_MOTORS];
    AHRS gyro_;

    // config
    PIDFController pid_conf_;

    frc::DifferentialDriveOdometry odom_;

private:
    // simulation stuff
    frc::sim::DifferentialDrivetrainSim sim_drive_train_;
    TalonFXSimCollection sim_left_, sim_right_;
    hal::SimDouble sim_gyro_yaw_;
    frc::Field2d sim_field_;

public:
    // Initialization
    DriveBase();

    // Live Reconfigure
    void Reconfigure();

    // Called by the scheduler every 20ms
    void Periodic() override;

    // used to update simulation
    void SimulationPeriodic() override;

public:
    units::meter_t GetX() const {return odom_.GetPose().X();}
    units::meter_t GetY() const {return odom_.GetPose().Y();}
    units::radian_t GetHeading() const {return odom_.GetPose().Rotation().Radians();}

public:
    void TankDrive(float left, float right);
    void ArcadeDrive(float v, float omega);

public:  // commands
    frc2::CommandPtr ArcadeCommand(const frc::Joystick &joystick);
};
