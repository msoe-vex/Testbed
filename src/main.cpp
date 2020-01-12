#include "main.h"

#include "TankOdometry.h"
#include "Math/Pose.h"
#include "PathManager.h"
#include "AdaptivePurePursuitController.h"
#include "CORETimer.h"

#include <string>
#include <fstream>
#include <streambuf>

using namespace std;

pros::ADIGyro * gyro;

lv_obj_t * robotImg;
lv_obj_t * output;

vector<Path> paths;

lv_point_t line_points[100];

LV_IMG_DECLARE(redField);
LV_IMG_DECLARE(robot);

double inchesToPixels(double inches) {
    return inches * (400.0/143.0);
}

double pixelsToInches(double pixels) {
    return pixels * (143.0/400.0);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	lv_obj_t * img1 = lv_img_create(lv_scr_act(), NULL);
	lv_img_set_src(img1, &redField);
	lv_obj_align(img1, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 0, 0);

	robotImg = lv_img_create(lv_scr_act(), NULL);
	lv_img_set_src(robotImg, &robot);
	lv_obj_align(robotImg, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 70, 0);

  output = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_align(output, LV_LABEL_ALIGN_LEFT);
  lv_obj_align(output, NULL, LV_ALIGN_IN_TOP_LEFT, 5, 5);

  lv_label_set_text(output, "Loading paths.......");

  printf("Opening file\n");

  PathManager::GetInstance()->LoadPathsFile("/usd/path.json");

  int numPaths = PathManager::GetInstance()->NumPaths();

  if(numPaths > 0) {
    lv_label_set_text(output, "Sucessfully loaded paths!");

    paths = PathManager::GetInstance()->GetPaths();
    auto waypoints = paths[0].getWaypoints();

    //lv_point_t line_points[] = {{0, 0}, {5, 5}, {70, 70}, {120, 10}, {180, 60}, {240, 10} };

    for(int i = 0; i < waypoints.size(); i++) {
      lv_point_t point;
      int x = inchesToPixels((143.0/2.0) + waypoints[i].getPoint().x());
      int y = inchesToPixels(waypoints[i].getPoint().y());

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
    lv_obj_t * line1;
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

  if(PathManager::GetInstance()->NumPaths() > 0) {
    auto path = paths[0];

    TankOdometry::GetInstance()->SetCurrentPose(Pose(path.getWaypoints()[0].getPoint(), Rotation2Dd(0)));

    AdaptivePurePursuitController controller(path, 4, 10.3, 1, 0.5);

    while (true) {
      int left = master.get_analog(ANALOG_LEFT_Y) * (12000.0/127.0);
  		int right = master.get_analog(ANALOG_RIGHT_X) * (12000.0/127.0);
  		Eigen::Rotation2Dd angle(toRadians(90 - gyro->get_value() * 0.1));
  		TankOdometry::GetInstance()->Update(left_mtr.getEncoder()->get(), right_mtr.getEncoder()->get(), 10.3);

  		Pose pose = TankOdometry::GetInstance()->GetPose();

      auto command = controller.Update(pose, pros::millis() / 1000.0);

  		printf("L: %2.2f R: %2.2f Time: %2.1f\n", command.leftVelocity, command.rightVelocity, pros::millis() / 1000.0);
  		// pros::lcd::print(0, "X: %2.1f Y: %2.1f Theta: %2.1f", pose.position.x(), pose.position.y(), toDegrees((double)pose.angle.angle()));
  		// pros::lcd::print(1, "L: %2.1f R: %2.1f Angle: %2.1f", left_mtr.getEncoder()->get(), right_mtr.getEncoder()->get(), angle);

  		left_mtr.moveVoltage(5.41/command.leftVelocity*12000);
  		right_mtr.moveVoltage(5.41/command.rightVelocity*12000);
  		pros::delay(20);

  		lv_obj_align(robotImg, NULL, LV_ALIGN_IN_BOTTOM_LEFT, inchesToPixels(pose.position.x() + (143.0/2.0)), -inchesToPixels(pose.position.y()));
  		lv_obj_set_top(robotImg, true);
    }

  }

	while (true) {
		int left = master.get_analog(ANALOG_LEFT_Y) * (12000.0/127.0);
		int right = master.get_analog(ANALOG_RIGHT_X) * (12000.0/127.0);
		Eigen::Rotation2Dd angle(toRadians(90 - gyro->get_value() * 0.1));
		TankOdometry::GetInstance()->Update(left_mtr.getEncoder()->get(), right_mtr.getEncoder()->get(), angle);

		Pose pose = TankOdometry::GetInstance()->GetPose();

		// printf("X: %2.1f Y: %2.1f Theta: %2.1f\n", pose.position.x(), pose.position.y(), pose.angle.angle());
		// pros::lcd::print(0, "X: %2.1f Y: %2.1f Theta: %2.1f", pose.position.x(), pose.position.y(), toDegrees((double)pose.angle.angle()));
		// pros::lcd::print(1, "L: %2.1f R: %2.1f Angle: %2.1f", left_mtr.getEncoder()->get(), right_mtr.getEncoder()->get(), angle);

		left_mtr.moveVoltage(left + right);
		right_mtr.moveVoltage(left - right);
		pros::delay(20);

		lv_obj_align(robotImg, NULL, LV_ALIGN_IN_BOTTOM_LEFT, inchesToPixels(pose.position.x() + (143.0/2.0)), -inchesToPixels(pose.position.y()));
		lv_obj_set_top(robotImg, true);
	}
}
