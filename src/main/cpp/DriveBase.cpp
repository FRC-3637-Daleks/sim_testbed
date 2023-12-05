#include "DriveBase.h"

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableBuilderImpl.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/RobotController.h>

#include <units/moment_of_inertia.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/mass.h>
#include <units/voltage.h>
#include <units/time.h>

#include <iostream>

using namespace units::literals;

namespace   // anonymous namespace
            // prevents these names from clashing with other cpp files
{
    constexpr int motor_ids[] = {0, 1, 2, 3};
    constexpr const char* motor_names[] = {"left_master", "left_follower", "right_master", "right_follower"};
    constexpr auto max_speed = 5.0_mps;
    constexpr auto moment_of_inertia = 2.1_kg_sq_m;
    constexpr auto mass = 26_kg;
    constexpr auto gyro_angle = 0.0_rad;
    constexpr auto wheel_radius = 3_in;
    constexpr auto wheel_distance = 1.0_m;
    constexpr auto gear_ratio = 8.0;
    constexpr auto pi = 3.1415926;
    constexpr auto wheel_circumference = 2.0*wheel_radius*pi;
    constexpr auto encoder_ticks_per_rev = 2048;
    constexpr auto encoder_ticks_per_meter = encoder_ticks_per_rev*gear_ratio / wheel_circumference;
    constexpr auto seconds_to_full_throttle = 0.2;  // clamps the motor acceleration

    constexpr auto kF_default = (1023*1.0/(max_speed*encoder_ticks_per_meter*0.1_s)).value();
    constexpr auto kP_default = 0.6;
    constexpr auto kI_default = 0.0;
    constexpr auto kD_default = 6.0;
}

DriveBase::DriveBase():
    motors_{motor_ids[0], motor_ids[1], motor_ids[2], motor_ids[3]},
    gyro_(frc::SPI::Port::kMXP),
    pid_conf_(kF_default, kP_default, kI_default, kD_default),
    odom_(gyro_angle, 0.0_m, 0.0_m),
    sim_drive_train_(
        frc::DCMotor::Falcon500(2),
        gear_ratio,
        moment_of_inertia,
        mass,
        wheel_radius,
        wheel_distance
    ),
    sim_left_(motors_[LEFT_MAIN].GetSimCollection()),
    sim_right_(motors_[RIGHT_MAIN].GetSimCollection()),
    sim_gyro_yaw_(HALSIM_GetSimValueHandle(HALSIM_GetSimDeviceHandle("navX-Sensor[4]"), "Yaw"))
{
    // Configures follower motors to output same power as main counterpart
    motors_[LEFT_FOLLOWER].Follow(motors_[LEFT_MAIN]);
    motors_[RIGHT_FOLLOWER].Follow(motors_[RIGHT_MAIN]);

    motors_[LEFT_MAIN].ConfigSelectedFeedbackSensor((FeedbackDevice)TalonFXFeedbackDevice::IntegratedSensor);
    motors_[RIGHT_MAIN].ConfigSelectedFeedbackSensor((FeedbackDevice)TalonFXFeedbackDevice::IntegratedSensor);

    motors_[LEFT_MAIN].ConfigClosedloopRamp(seconds_to_full_throttle);
    motors_[RIGHT_MAIN].ConfigClosedloopRamp(seconds_to_full_throttle);

    frc::SmartDashboard::PutData("drive_pid", &pid_conf_);
    Reconfigure();

    TankDrive(0, 0);
    
    this->SetName("DriveBase");
    for (int i = 0; i < N_MOTORS; i++) AddChild(motor_names[i], &motors_[i]);

    frc::SmartDashboard::PutData("Sim Field", &sim_field_);
    gyro_.ZeroYaw();

    std::cout << "DriveBase Constructor Complete!" << std::endl;
}

void DriveBase::Reconfigure()
{
    motors_[LEFT_MAIN].Config_kF(0, pid_conf_.GetF());
    motors_[RIGHT_MAIN].Config_kF(0, pid_conf_.GetF());
    
    motors_[LEFT_MAIN].Config_kP(0, pid_conf_.GetP());
    motors_[RIGHT_MAIN].Config_kP(0, pid_conf_.GetP());

    motors_[LEFT_MAIN].Config_kI(0, pid_conf_.GetI());
    motors_[RIGHT_MAIN].Config_kI(0, pid_conf_.GetI());

    motors_[LEFT_MAIN].Config_kD(0, pid_conf_.GetD());
    motors_[RIGHT_MAIN].Config_kD(0, pid_conf_.GetD());
}

void DriveBase::Periodic()
{
    odom_.Update(
        gyro_.GetYaw()*1_rad,
        motors_[LEFT_MAIN].GetSelectedSensorVelocity() / encoder_ticks_per_meter,
        motors_[RIGHT_MAIN].GetSelectedSensorVelocity() / encoder_ticks_per_meter
    );

    frc::SmartDashboard::PutNumber(
        "left vel", motors_[LEFT_MAIN].GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber(
        "right vel", motors_[RIGHT_MAIN].GetSelectedSensorVelocity());
    
    frc::SmartDashboard::PutNumber(
        "left target vel", motors_[LEFT_MAIN].GetClosedLoopTarget());
    frc::SmartDashboard::PutNumber(
        "right target vel", motors_[RIGHT_MAIN].GetClosedLoopTarget());
    
    frc::SmartDashboard::PutNumber(
        "heading", gyro_.GetYaw());
}

void DriveBase::SimulationPeriodic()
{
    sim_left_.SetBusVoltage(frc::RobotController::GetBatteryVoltage().value());
    sim_right_.SetBusVoltage(frc::RobotController::GetBatteryVoltage().value());

    sim_drive_train_.SetInputs(
        sim_left_.GetMotorOutputLeadVoltage()*1.0_V,
        sim_right_.GetMotorOutputLeadVoltage()*1.0_V);
    
    sim_drive_train_.Update(20_ms);
    sim_field_.SetRobotPose(sim_drive_train_.GetPose());

    sim_left_.SetIntegratedSensorRawPosition(
        sim_drive_train_.GetLeftPosition()*encoder_ticks_per_meter
    );

    sim_left_.SetIntegratedSensorVelocity(
        sim_drive_train_.GetLeftVelocity()*encoder_ticks_per_meter*0.1_s
    );

    sim_right_.SetIntegratedSensorRawPosition(
        sim_drive_train_.GetRightPosition()*encoder_ticks_per_meter
    );

    sim_right_.SetIntegratedSensorVelocity(
        sim_drive_train_.GetRightVelocity()*encoder_ticks_per_meter*0.1_s
    );

    sim_gyro_yaw_.Set(sim_drive_train_.GetHeading().Radians().value());
}

void DriveBase::TankDrive(float left, float right)
{
    auto left_vel = left*max_speed;
    auto right_vel = right*max_speed;
    
    motors_[LEFT_MAIN].Set(ControlMode::Velocity, left_vel*encoder_ticks_per_meter*0.1_s);
    motors_[RIGHT_MAIN].Set(ControlMode::Velocity, right_vel*encoder_ticks_per_meter*0.1_s);
}

void DriveBase::ArcadeDrive(float v, float omega)
{
    TankDrive(v - omega, v + omega);
}

frc2::CommandPtr DriveBase::ArcadeCommand(const frc::Joystick &joystick)
{
    return this->Run(
        [this, &joystick]() {ArcadeDrive(joystick.GetY(), -joystick.GetX());}
    ).WithName("Arcade Drive");
}

void PIDFController::InitSendable(wpi::SendableBuilder& builder)
{
    // call parent first
    PIDController::InitSendable(builder);

    builder.SetSmartDashboardType("PIDFController");

    // add f gain
    builder.AddDoubleProperty("f", [this]{return kF_;}, [this](double kF){kF_ = kF;});
}
