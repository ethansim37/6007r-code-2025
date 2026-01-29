#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include <cstddef>



	pros::Controller master(pros::E_CONTROLLER_MASTER);


  pros::adi::Pneumatics hood(2, true);
  pros::adi::Pneumatics intakeHood(1, true);
	pros::MotorGroup left_mg({-16, -3, -15});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({9, 18, 19});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6

  pros::Motor intake(-2);
  pros::Motor upperIntake(13);

  pros::IMU imu(7);  // Inertial sensor on port 7

  lemlib::Drivetrain drivetrain(&left_mg, &right_mg, 11.5, lemlib::Omniwheel::NEW_325, 450, 8);

  lemlib::ControllerSettings lateralSettings(6, 0, 7, 0.45, 1.3, 300, 0.8, 250,
                                           18);


lemlib::ControllerSettings
    angularSettings(2.2,    // proportional gain (kP)
                    0.0015, // integral gain (kI)
                    14.5,   // derivative gain (kD)
                    6.0,    // anti windup
                    0.5,    // small error range, in inches
                    150,    // small error range timeout, in milliseconds
                    0,      // large error range, in inches
                    0,      // large error range timeout, in milliseconds
                    0       // maximum acceleration (slew)
    );


lemlib::OdomSensors sensors(NULL, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            NULL, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
  lemlib::Chassis chassis(drivetrain, lateralSettings, angularSettings, sensors);





/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
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
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);



  pros::delay(5000);
  autonomous(); //Emulate autonomous mode

  





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
  chassis.setPose((16.88+7), 79.64, 180, false);// robot starts with right side flush against 
                                                // parking barrier's side opposite the wall, 
                                                // with the very back of the robot lined up 
                                                // with the back of the red piece
  chassis.moveToPoint((23.44-8), 79.64, 5);//Move in front of the loader
  chassis.turnToHeading(270, 2);//Turn to face the loader
  intakeHood.extend();//Extend intake hood to prepare for intake

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

	while (true) {
        chassis.tank(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake.move_voltage(12000);
      upperIntake.move_voltage(5000);
      hood.retract();

    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intake.move_voltage(12000);
      upperIntake.move_voltage(12000);
      hood.extend();
    
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
      intakeHood.extend();

    } else {
      intake.move_voltage(0);
      upperIntake.move_voltage(0);
      hood.retract(); //Was indented before I edit, I think its typo so I unindented it
      intakeHood.retract();
    }
	}
}