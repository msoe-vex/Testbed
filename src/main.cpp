#include "main.h"

using namespace std;

chassis chassis(constants::LEFT_FRONT_DRIVE_PORT, constants::LEFT_REAR_DRIVE_PORT,
		            constants::RIGHT_FRONT_DRIVE_PORT, constants::RIGHT_REAR_DRIVE_PORT);

lift lift(constants::LEFT_LIFT_PORT, constants::RIGHT_LIFT_PORT,
	        constants::LIFT_LIMIT_PORT);

intake intake(constants::LEFT_INTAKE_PORT, constants::RIGHT_INTAKE_PORT, constants::TRAY_TILT_PORT);

TestAuton testAuton(&chassis);

/* BAD PORTS */

pros::ADIGyro *gyro;

lv_obj_t *robotImg;
lv_obj_t *output;

unordered_map<string, Path> paths;

lv_point_t line_points[100];

LV_IMG_DECLARE(redField);
LV_IMG_DECLARE(robot);

double inchesToPixels(double inches) {
    return inches * (400.0 / 143.0);
}

double pixelsToInches(double pixels) {
    return pixels * (143.0 / 400.0);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    lv_obj_t *img1 = lv_img_create(lv_scr_act(), NULL);
    lv_img_set_src(img1, &redField);
    lv_obj_align(img1, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 0, 0);

    robotImg = lv_img_create(lv_scr_act(), NULL);
    lv_img_set_src(robotImg, &robot);
    lv_obj_align(robotImg, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 70, 0);

    output = lv_label_create(lv_scr_act(), NULL);
    lv_label_set_align(output, LV_LABEL_ALIGN_LEFT);
    lv_obj_align(output, NULL, LV_ALIGN_IN_TOP_LEFT, 5, 5);

    lv_label_set_text(output, "Loading paths.......");

    printf("Loading paths.......");

    PathManager::GetInstance()->LoadPathsText(pathText);

    int numPaths = PathManager::GetInstance()->NumPaths();

    printf("Loaded %i paths", numPaths);

    if (numPaths > 0) {
        lv_label_set_text(output, "Sucessfully loaded paths!");

        paths = PathManager::GetInstance()->GetPaths();
        auto waypoints = paths["WallToCubeStack3"].getWaypoints();

        //lv_point_t line_points[] = {{0, 0}, {5, 5}, {70, 70}, {120, 10}, {180, 60}, {240, 10} };

        for (int i = 0; i < waypoints.size(); i++) {
            lv_point_t point;
            int x = inchesToPixels((143.0 / 2.0) + waypoints[i].position.getX());
            int y = inchesToPixels(waypoints[i].position.getY());

            point.x = x;
            point.y = y;

            line_points[i] = point;

            printf("X: %i Y: %i\n", x, y);
        }

        static lv_style_t style_line;
        lv_style_copy(&style_line, &lv_style_plain);
        style_line.line.color = LV_COLOR_CYAN;
        style_line.line.width = 2;
        style_line.line.rounded = 1;

        /*Copy the previous line and apply the new style*/
        lv_obj_t *line1;
        line1 = lv_line_create(lv_scr_act(), NULL);
        lv_line_set_points(line1, line_points, waypoints.size());     /*Set the points*/
        lv_line_set_style(line1, &style_line);
        lv_line_set_y_invert(line1, true);
        lv_obj_set_top(line1, true);
        lv_obj_align(line1, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 0, 0);
    } else {
        lv_label_set_text(output, "Failed to load paths");
    }

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
void autonomous() {
    testAuton.AutonInit();

    while(!testAuton.Complete()) {
        chassis.periodic();
        testAuton.Auton();
        pros::delay(10);
    }
}

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
		chassis.periodic();

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

		auto pose = TankOdometry::GetInstance()->GetPose();

		lv_obj_align(robotImg, NULL, LV_ALIGN_IN_BOTTOM_LEFT, inchesToPixels(pose.position.x() + (143.0 / 2.0)),
								 -inchesToPixels(pose.position.y()));
		lv_obj_set_top(robotImg, true);

		pros::delay(20);
	}
}
