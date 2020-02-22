#include "Chassis.h"
#include "TankOdometry.h"

Chassis::Chassis(int leftFrontDriveMotorPort, int leftRearDriveMotorPort,
                 int rightFrontDriveMotorPort, int rightRearDriveMotorPort) {
  leftFrontDriveMotor = new pros::Motor(leftFrontDriveMotorPort, false);
  leftRearDriveMotor = new pros::Motor(leftRearDriveMotorPort, false);
  rightFrontDriveMotor = new pros::Motor(rightFrontDriveMotorPort, true);
  rightRearDriveMotor = new pros::Motor(rightRearDriveMotorPort, true);

    leftFrontDriveMotor->tare_position();
    leftRearDriveMotor->tare_position();
    rightFrontDriveMotor->tare_position();
    rightRearDriveMotor->tare_position();

    TankOdometry::EncoderConfig encoderConfig;
    encoderConfig.initialTicks = 0;
    encoderConfig.ticksPerWheelRevolution = 900;
    encoderConfig.wheelDiameter = 4;

    TankOdometry::GetInstance()->Initialize(encoderConfig, encoderConfig);
}

void Chassis::Periodic() {
    TankOdometry::GetInstance()->Update(leftFrontDriveMotor->get_position(), rightFrontDriveMotor->get_position(), 10.5);
}

void Chassis::SetSpeed(int leftDriveSpeed, int rightDriveSpeed) {
  leftFrontDriveMotor->move(leftDriveSpeed);
  leftRearDriveMotor->move(leftDriveSpeed);
  rightFrontDriveMotor->move(rightDriveSpeed);
  rightRearDriveMotor->move(rightDriveSpeed);
}

double Chassis::GetLeftSpeed() {
  return (leftFrontDriveMotor->get_position() + leftRearDriveMotor->get_position()) / 2;
}

double Chassis::GetRightSpeed() {
  return (rightFrontDriveMotor->get_position() + rightRearDriveMotor->get_position()) / 2;
}

void Chassis::SetPIDPosition(double position) {
  leftFrontDriveMotor->move_absolute(position, 150);
  leftRearDriveMotor->move_absolute(position, 150);
  rightFrontDriveMotor->move_absolute(position, 150);
  rightRearDriveMotor->move_absolute(position, 150);
}

Chassis::~Chassis() { // Deconstructor
  free(leftFrontDriveMotor); // Free memory
  free(leftRearDriveMotor); // Free memory
  free(rightFrontDriveMotor); // Free memory
  free(rightRearDriveMotor); // Free memory

  leftFrontDriveMotor = 0; // Clear reference
  leftRearDriveMotor = 0; // Clear reference
  rightFrontDriveMotor = 0; // Clear reference
  rightRearDriveMotor = 0;
}
/*
void Chassis::SetPIDPosition(double position) {
  leftFrontDriveMotor->move_absolute(position, constants::LIFT_MAX_VEL);
  leftRearDriveMotor->move_absolute(position, constants::LIFT_MAX_VEL);
  rightFrontDriveMotor->move_absolute(position, constants::LIFT_MAX_VEL);
  rightRearDriveMotor->move_absolute(position, constants::LIFT_MAX_VEL);
}

void Chassis::SetPIDVelocity(double liftVelocity) {
  leftFrontDriveMotor->move_velocity(liftVelocity);
  leftRearDriveMotor->move_velocity(liftVelocity);
  rightFrontDriveMotor->move_velocity(liftVelocity);
  rightRearDriveMotor-->move_velocity(liftVelocity);
}
*/
