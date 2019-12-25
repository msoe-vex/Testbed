#include "main.h"

#include "TankOdometry.h"

pros::ADIGyro * gyro;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	gyro = new pros::ADIGyro('A');
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	okapi::Motor left_mtr(11);
	okapi::Motor right_mtr(12);

	right_mtr.setReversed(true);

	TankOdometry::EncoderConfig encoderConfig;
	encoderConfig.initialTicks = 0;
	encoderConfig.ticksPerWheelRevolution = 360;
	encoderConfig.wheelDiameter = 3.25;


	TankOdometry::GetInstance()->Initialize(encoderConfig, encoderConfig);

	while (true) {
		int left = master.get_analog(ANALOG_LEFT_Y) * (12000.0/127.0);
		int right = master.get_analog(ANALOG_RIGHT_X) * (12000.0/127.0);
		Eigen::Rotation2Dd angle(toRadians(90 - gyro->get_value() * 0.1));
		TankOdometry::GetInstance()->Update(left_mtr.getEncoder()->get(), right_mtr.getEncoder()->get(), angle);

		Pose pose = TankOdometry::GetInstance()->GetPose();

		printf("X: %2.1f Y: %2.1f Theta: %2.1f\n", pose.position.x(), pose.position.y(), pose.angle.angle());
		pros::lcd::print(0, "X: %2.1f Y: %2.1f Theta: %2.1f", pose.position.x(), pose.position.y(), toDegrees((double)pose.angle.angle()));
		pros::lcd::print(1, "L: %2.1f R: %2.1f Angle: %2.1f", left_mtr.getEncoder()->get(), right_mtr.getEncoder()->get(), angle);

		left_mtr.moveVoltage(left + right);
		right_mtr.moveVoltage(left - right);
		pros::delay(20);
	}
}
