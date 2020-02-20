#include "main.h"

#include "TankOdometry.h"
#include "Math/Pose.h"
#include "PathManager.h"
#include "AdaptivePursuit.h"
#include "CORETimer.h"

#include <string>
#include <fstream>
#include <streambuf>

#include "Auton.h"

using namespace std;

//Temp store path as string since sd card got lost :(
string thing = "{\n"
               "    \"sharedWaypoints\": [],\n"
               "    \"paths\": [\n"
               "        {\n"
               "            \"name\": \"FirstPath\",\n"
               "            \"waypoints\": [\n"
               "                {\n"
               "                    \"name\": \"startWaypoint\",\n"
               "                    \"angle\": 0,\n"
               "                    \"x\": 0,\n"
               "                    \"y\": 7.5,\n"
               "                    \"shared\": false\n"
               "                },\n"
               "                {\n"
               "                    \"name\": \"endWaypoint\",\n"
               "                    \"angle\": 0,\n"
               "                    \"x\": 0,\n"
               "                    \"y\": 71,\n"
               "                    \"shared\": false\n"
               "                }\n"
               "            ],\n"
               "            \"points\": [\n"
               "                {\n"
               "                    \"x\": 0,\n"
               "                    \"y\": 7.5,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 0,\n"
               "                    \"y\": 11.47,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 0,\n"
               "                    \"y\": 15.44,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 0,\n"
               "                    \"y\": 19.41,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 0,\n"
               "                    \"y\": 23.38,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 0,\n"
               "                    \"y\": 27.34,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 0,\n"
               "                    \"y\": 31.31,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 0,\n"
               "                    \"y\": 35.28,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 0,\n"
               "                    \"y\": 39.25,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 0,\n"
               "                    \"y\": 43.22,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 0,\n"
               "                    \"y\": 47.19,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 0,\n"
               "                    \"y\": 51.16,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 0,\n"
               "                    \"y\": 55.13,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 0,\n"
               "                    \"y\": 59.09,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 0,\n"
               "                    \"y\": 63.06,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 0,\n"
               "                    \"y\": 67.03,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 0,\n"
               "                    \"y\": 71,\n"
               "                    \"speed\": 5.41\n"
               "                }\n"
               "            ]\n"
               "        },\n"
               "        {\n"
               "            \"name\": \"TestPath\",\n"
               "            \"waypoints\": [\n"
               "                {\n"
               "                    \"name\": \"start\",\n"
               "                    \"angle\": 0,\n"
               "                    \"x\": 13.72,\n"
               "                    \"y\": 14.18,\n"
               "                    \"shared\": false\n"
               "                },\n"
               "                {\n"
               "                    \"name\": \"mid\",\n"
               "                    \"angle\": 21.84,\n"
               "                    \"x\": 33.45,\n"
               "                    \"y\": 45.16,\n"
               "                    \"shared\": false\n"
               "                },\n"
               "                {\n"
               "                    \"name\": \"end\",\n"
               "                    \"angle\": -90,\n"
               "                    \"x\": 17.57,\n"
               "                    \"y\": 59.81,\n"
               "                    \"shared\": false\n"
               "                },\n"
               "                {\n"
               "                    \"name\": \"wp\",\n"
               "                    \"angle\": 180,\n"
               "                    \"x\": -0.31,\n"
               "                    \"y\": 37.76,\n"
               "                    \"shared\": false\n"
               "                },\n"
               "                {\n"
               "                    \"name\": \"wp\",\n"
               "                    \"angle\": 180,\n"
               "                    \"x\": -0.31,\n"
               "                    \"y\": 28.05,\n"
               "                    \"shared\": false\n"
               "                }\n"
               "            ],\n"
               "            \"points\": [\n"
               "                {\n"
               "                    \"x\": 13.72,\n"
               "                    \"y\": 14.18,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 14.08,\n"
               "                    \"y\": 17.91,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 15.06,\n"
               "                    \"y\": 21.24,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 16.57,\n"
               "                    \"y\": 24.24,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 18.48,\n"
               "                    \"y\": 26.98,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 20.68,\n"
               "                    \"y\": 29.54,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 23.05,\n"
               "                    \"y\": 31.99,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 25.47,\n"
               "                    \"y\": 34.41,\n"
               "                    \"speed\": 5.27\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 27.84,\n"
               "                    \"y\": 36.86,\n"
               "                    \"speed\": 4.94\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 30.04,\n"
               "                    \"y\": 39.42,\n"
               "                    \"speed\": 4.59\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 31.95,\n"
               "                    \"y\": 42.16,\n"
               "                    \"speed\": 4.21\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 33.45,\n"
               "                    \"y\": 45.16,\n"
               "                    \"speed\": 3.79\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 33.45,\n"
               "                    \"y\": 45.16,\n"
               "                    \"speed\": 3.79\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 34.75,\n"
               "                    \"y\": 48.84,\n"
               "                    \"speed\": 3.23\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 35.45,\n"
               "                    \"y\": 51.88,\n"
               "                    \"speed\": 2.71\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 35.62,\n"
               "                    \"y\": 54.34,\n"
               "                    \"speed\": 2.21\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 35.31,\n"
               "                    \"y\": 56.28,\n"
               "                    \"speed\": 1.71\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 34.56,\n"
               "                    \"y\": 57.75,\n"
               "                    \"speed\": 1.39\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 33.44,\n"
               "                    \"y\": 58.81,\n"
               "                    \"speed\": 1.6\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 32.01,\n"
               "                    \"y\": 59.53,\n"
               "                    \"speed\": 2.27\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 30.3,\n"
               "                    \"y\": 59.96,\n"
               "                    \"speed\": 3.84\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 28.39,\n"
               "                    \"y\": 60.16,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 26.32,\n"
               "                    \"y\": 60.19,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 24.15,\n"
               "                    \"y\": 60.11,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 21.93,\n"
               "                    \"y\": 59.98,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 19.72,\n"
               "                    \"y\": 59.86,\n"
               "                    \"speed\": 5.38\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 17.57,\n"
               "                    \"y\": 59.81,\n"
               "                    \"speed\": 5.18\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 17.57,\n"
               "                    \"y\": 59.81,\n"
               "                    \"speed\": 5.18\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 17.57,\n"
               "                    \"y\": 59.81,\n"
               "                    \"speed\": 5.18\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 13.77,\n"
               "                    \"y\": 59.57,\n"
               "                    \"speed\": 4.8\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 10.53,\n"
               "                    \"y\": 58.88,\n"
               "                    \"speed\": 4.44\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 7.8,\n"
               "                    \"y\": 57.76,\n"
               "                    \"speed\": 4.09\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 5.56,\n"
               "                    \"y\": 56.26,\n"
               "                    \"speed\": 3.75\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 3.75,\n"
               "                    \"y\": 54.4,\n"
               "                    \"speed\": 4\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 2.33,\n"
               "                    \"y\": 52.23,\n"
               "                    \"speed\": 4.52\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 1.27,\n"
               "                    \"y\": 49.77,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 0.52,\n"
               "                    \"y\": 47.06,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": 0.03,\n"
               "                    \"y\": 44.13,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": -0.23,\n"
               "                    \"y\": 41.02,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": -0.31,\n"
               "                    \"y\": 37.76,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": -0.31,\n"
               "                    \"y\": 37.76,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": -0.31,\n"
               "                    \"y\": 36.37,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": -0.31,\n"
               "                    \"y\": 34.99,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": -0.31,\n"
               "                    \"y\": 33.6,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": -0.31,\n"
               "                    \"y\": 32.21,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": -0.31,\n"
               "                    \"y\": 30.82,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": -0.31,\n"
               "                    \"y\": 29.44,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": -0.31,\n"
               "                    \"y\": 28.05,\n"
               "                    \"speed\": 5.41\n"
               "                },\n"
               "                {\n"
               "                    \"x\": -0.31,\n"
               "                    \"y\": 28.05,\n"
               "                    \"speed\": 5.41\n"
               "                }\n"
               "            ]\n"
               "        }\n"
               "    ]\n"
               "}";

pros::ADIGyro *gyro;

lv_obj_t *robotImg;
lv_obj_t *output;

vector <Path> paths;

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

    printf("Opening file\n");

    PathManager::GetInstance()->LoadPathsText(thing);

    int numPaths = PathManager::GetInstance()->NumPaths();

    if (numPaths > 0) {
        lv_label_set_text(output, "Sucessfully loaded paths!");

        paths = PathManager::GetInstance()->GetPaths();
        auto waypoints = paths[0].getWaypoints();

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

    okapi::Motor frontLeft(11);
    okapi::Motor backLeft(12);
    okapi::Motor frontRight(13);
    okapi::Motor backRight(14);

    frontRight.setReversed(true);
    backRight.setReversed(true);

    TankOdometry::EncoderConfig encoderConfig;
    encoderConfig.initialTicks = 0;
    encoderConfig.ticksPerWheelRevolution = 360;
    encoderConfig.wheelDiameter = 3.25;

    TankOdometry::GetInstance()->Initialize(encoderConfig, encoderConfig,
                                            Pose(Vector2d(paths[1].getFirstWaypoint().position.getY(),
                                                          -paths[1].getFirstWaypoint().position.getX()),
                                                 Rotation2Dd(0)));

    if (PathManager::GetInstance()->NumPaths() > 0) {
        auto path = paths[0];

        AdaptivePursuit controller(10, 10, 0.01, paths[1], false, 0.01, true, 3.25);

        //Path following loop
        while (true) {
            Eigen::Rotation2Dd angle(toRadians(gyro->get_value() * 0.1));
            TankOdometry::GetInstance()->Update(frontLeft.getEncoder()->get(), frontRight.getEncoder()->get(),
                                                angle); //Using pure encoder based odometry as gyro odometry seems to be having issues

            auto command = controller.Update(TankOdometry::GetInstance()->GetPose(), pros::millis() / 1000.0);


            //10.3 is the small robot track width

            // printf("L: %2.2f R: %2.2f LE: %2.2f RE: %2.2f LC: %2.2f RC: %2.2f A: %2.2f Time: %2.1f\n", command.left, command.right, frontLeft.getEncoder()->get(), frontRight.getEncoder()->get(), command.left, command.right, angle.angle() * 180/3.141592, pros::millis() / 1000.0);
            // pros::lcd::print(0, "X: %2.1f Y: %2.1f Theta: %2.1f", pose.position.x(), pose.position.y(), toDegrees((double)pose.angle.angle()));
            // pros::lcd::print(1, "L: %2.1f R: %2.1f Angle: %2.1f", left_mtr.getEncoder()->get(), right_mtr.getEncoder()->get(), angle);

            frontLeft.moveVoltage((int) command.left * (12000.0 / 5.41));
            backLeft.moveVoltage((int) command.left * (12000.0 / 5.41));
            frontRight.moveVoltage((int) command.right * (12000.0 / 5.41));
            backRight.moveVoltage((int) command.right * (12000.0 / 5.41));

            pros::delay(10);

            // lv_obj_align(robotImg, NULL, LV_ALIGN_IN_BOTTOM_LEFT, inchesToPixels(TankOdometry::GetInstance()->GetPose().position.x() + (143.0/2.0)), -inchesToPixels(TankOdometry::GetInstance()->GetPose().position.y()));
            // lv_obj_set_top(robotImg, true);
        }

    }


    //Fallback driver code
    while (true) {
        int left = master.get_analog(ANALOG_LEFT_Y) * (12000.0 / 127.0);
        int right = master.get_analog(ANALOG_RIGHT_X) * (12000.0 / 127.0);
        Eigen::Rotation2Dd angle(toRadians(90 - gyro->get_value() * 0.1));
        TankOdometry::GetInstance()->Update(frontLeft.getEncoder()->get(), frontRight.getEncoder()->get(), angle);

        Pose pose = TankOdometry::GetInstance()->GetPose();

        // printf("X: %2.1f Y: %2.1f Theta: %2.1f\n", pose.position.x(), pose.position.y(), pose.angle.angle());
        // pros::lcd::print(0, "X: %2.1f Y: %2.1f Theta: %2.1f", pose.position.x(), pose.position.y(), toDegrees((double)pose.angle.angle()));
        // pros::lcd::print(1, "L: %2.1f R: %2.1f Angle: %2.1f", left_mtr.getEncoder()->get(), right_mtr.getEncoder()->get(), angle);

        backLeft.moveVoltage(left + right);
        frontLeft.moveVoltage(left + right);
        backRight.moveVoltage(left - right);
        frontRight.moveVoltage(left - right);

        pros::delay(20);

        lv_obj_align(robotImg, NULL, LV_ALIGN_IN_BOTTOM_LEFT, inchesToPixels(pose.position.x() + (143.0 / 2.0)),
                     -inchesToPixels(pose.position.y()));
        lv_obj_set_top(robotImg, true);
    }
}
