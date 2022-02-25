#include "main.h"
#include <stdio.h>
#include "okapi/api.hpp"
#include "okapi/api/units/RQuantity.hpp"
#include <stdlib.h>

using namespace okapi::literals; //use okapi namespace for literals

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(3, "slayyyyy");

	pros::lcd::register_btn1_cb(on_center_button);

}
//define motors and sensors
#define F_LEFT_WHEEL_PORT 1 //front left drive
#define F_RIGHT_WHEEL_PORT 2 //front right drive
#define R_LEFT_WHEEL_PORT 3 //rear left drive
#define R_RIGHT_WHEEL_PORT 4 //rear right drive
#define L_LIFT_PORT 7 // left Lift
#define R_LIFT_PORT 8 // right lift
#define CLAW_PORT 10 //claw
#define TILTER_PORT 9 //back tilter
#define CLAWPOTENTIOMETER_PORT 'A'
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
	pros::Motor claw (CLAW_PORT, MOTOR_GEARSET_18); //claw identifier
	pros::ADIAnalogIn sensor (CLAWPOTENTIOMETER_PORT);
	pros::Motor motor (CLAW_PORT);
	//chassis motors
	pros::Motor f_left_wheel (F_LEFT_WHEEL_PORT, MOTOR_GEARSET_6, true);
	pros::Motor f_right_wheel (F_RIGHT_WHEEL_PORT, MOTOR_GEARSET_6);
	pros::Motor r_left_wheel (R_LEFT_WHEEL_PORT, MOTOR_GEARSET_6, true);
	pros::Motor r_right_wheel (R_RIGHT_WHEEL_PORT, MOTOR_GEARSET_6);

	sensor.calibrate();

	std::shared_ptr<okapi::OdomChassisController> chassis =
	okapi::ChassisControllerBuilder()
	.withMotors(1, 2, 4, 3) //upper left, upper right, lower right, lower left, all reversed bc geared
	//front 2 chassis motors, 1 and 2, both reversed
	.withDimensions(okapi::AbstractMotor::gearset::blue, {{1.82_in, 15.25_in}, okapi::imev5BlueTPR})
	//blue gearset, 3.25 inch wheels with 3:5 gear ratio makes effective diameter 2.34, 15.25 in track
	.withOdometry()
	.buildOdometry(); //build odometry chassis


	std::shared_ptr<okapi::OdomChassisController> invChassis =
	okapi::ChassisControllerBuilder()
	.withMotors(-1, -2, -4, -3)
	.withDimensions(okapi::AbstractMotor::gearset::blue, {{1.82_in, 15.25_in}, okapi::imev5BlueTPR})
	//blue gearset, 3.25 inch wheels with 3:5 gear ratio makes effective diameter 2.34, 15.25 in track
	.withOdometry()
	.buildOdometry();
	//odometry does not drive backwards, so use this to reverse

	//chassis->turnToAngle(90_deg); turning sample
	chassis->setState({0.0_in, 0.0_in, 0_deg});
	invChassis->setState({0_in, 0_in, 0_deg});
	//set position and angle to 0,0,0
	chassis->driveToPoint({44_in, 0_ft});

	while(sensor.get_value() < 2048) {
		claw.move_velocity(-100);
		if ( sensor.get_value() < 1500){
			break;
		}
	}

	invChassis->driveToPoint({-20_in, 0_in});
		//move backwards
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

	//motor controller stuff
	pros::Motor f_left_wheel (F_LEFT_WHEEL_PORT, MOTOR_GEARSET_6, true);
	pros::Motor f_right_wheel (F_RIGHT_WHEEL_PORT, MOTOR_GEARSET_6);
	pros::Motor r_left_wheel (R_LEFT_WHEEL_PORT, MOTOR_GEARSET_6, true);
	pros::Motor r_right_wheel (R_RIGHT_WHEEL_PORT, MOTOR_GEARSET_6);
	pros::Motor leftArm (L_LIFT_PORT, MOTOR_GEARSET_36);
	pros::Motor rightArm(R_LIFT_PORT, MOTOR_GEARSET_36, true);
	pros::Motor claw (CLAW_PORT, MOTOR_GEARSET_18);
	pros::Motor tilter (TILTER_PORT, true);

	pros::Controller master (CONTROLLER_MASTER);

	while (true) {
		int power = master.get_analog(ANALOG_LEFT_Y); //left analog stick
		int turn = master.get_analog(ANALOG_LEFT_X);
		int left = power + turn;
		int right = power - turn;
		f_left_wheel.move(left); //chassis motors
		f_right_wheel.move(right);
		r_left_wheel.move(left);
		r_right_wheel.move(right);

		if (master.get_digital(DIGITAL_R1)) { //lift
			leftArm.move_velocity(100);
			rightArm.move_velocity(100);
		}
		else if (master.get_digital(DIGITAL_R2)) {
			leftArm.move_velocity(-100);
			rightArm.move_velocity(-100);
		}
		else {
			leftArm.move_velocity(0);
			rightArm.move_velocity(0);
		}

		if(master.get_digital(DIGITAL_A)) { //claw
			claw.move_velocity(100);
		}
		else if (master.get_digital(DIGITAL_B)) {
			claw.move_velocity(-100);
		}
		else {
			claw.move_velocity(0);
		}

		if(master.get_digital(DIGITAL_X)) { //dick
			tilter.move_velocity(100);
		}
		else if (master.get_digital(DIGITAL_Y)) {
			tilter.move_velocity(-100);
		}
		else {
			tilter.move_velocity(0);
		}

		pros::delay(2);

	}
}
