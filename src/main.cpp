#include "main.h"

chassis chassis(constants::LEFT_FRONT_DRIVE_PORT, constants::LEFT_REAR_DRIVE_PORT,
		            constants::RIGHT_FRONT_DRIVE_PORT, constants::RIGHT_REAR_DRIVE_PORT);

lift lift(constants::LEFT_LIFT_PORT, constants::RIGHT_LIFT_PORT,
	        constants::LIFT_LIMIT_PORT);

intake intake(constants::LEFT_INTAKE_PORT, constants::RIGHT_INTAKE_PORT, constants::TRAY_TILT_PORT);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
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

	while (true) {
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = master.get_analog(ANALOG_RIGHT_Y);
		chassis.setSpeed(left, right);

		lift.manualControl(master);

		lift.periodic();

		if (master.get_digital(DIGITAL_R1) == 1) {
			intake.setSpeed(127, 127);
		} else if (master.get_digital(DIGITAL_R2) == 1) {
			intake.setSpeed(-127, -127);
		} else {
			intake.setSpeed(0, 0);
		}

		if (master.get_digital(DIGITAL_X) == 1) {
			intake.pivot(127);
		} else if (master.get_digital(DIGITAL_B) == 1) {
			intake.pivot(-127);
		} else {
			intake.pivot(0);
		}

		pros::delay(20);
	}
}
