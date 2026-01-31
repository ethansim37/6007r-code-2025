#include "main.h"
#include "lemlib/api.hpp"
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

  pros::Rotation horizontal_encoder(1);


  pros::adi::Pneumatics hood(2, true);
  pros::adi::Pneumatics intakeHood(1, true);
	pros::MotorGroup left_mg({-16, -3, -15});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({9, 18, 19});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6

  pros::Motor intake(-2);
  pros::Motor upperIntake(7);

  pros::IMU imu(13);  // Inertial sensor on port 7

  lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -2.25);

  lemlib::Drivetrain drivetrain(&left_mg, &right_mg, 11.5, lemlib::Omniwheel::NEW_325, 450, 8);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
  lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);





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
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });

  





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
void brainColor(uint32_t color){
  pros::screen::set_pen(color);
  pros::screen::fill_rect(0, 0, 480, 240); // full Brain screen
}

void autonomous() {
  brainColor(0x000000); //red


  //Pick up BALLS (step 1)
  chassis.setPose((16.88+7), 60.77, 0, false); //robot starts with right side flush against 
                                                 // parking barrier's side opposite the wall, 
                                                 // with the very back of the robot lined up 
                                                 // with the back of the red piece
  intakeHood.extend();//Extend intake hood to prepare for intake
  hood.retract();//Ensure hood is retracted so no BALLS fly away
  chassis.moveToPose(16.88+7, 116.97-5, -90, 2000);//Move in front the loader
  chassis.moveToPose(23-20, 116.97-5, -90, 2000);//Move into the loader to intake BALLS
  intake.move_voltage(12000);
  for(int i = 0; i < 5; i++){
    chassis.moveToPose(23-19, 116.97-5, -90, 1000);//Ensure all BALLS are intaken
  }
  intake.move_voltage(0);//end intake
  chassis.moveToPose(23+22, 116.97-6, -90, 3000,{.forwards = false});//Back away from loader

  // Drop balls into goal (step 2)
  pros::delay(1000);
  hood.extend();//Extend hood to prepare for outtake
  intake.move_voltage(12000);//Outtake
  upperIntake.move_voltage(12000);//Outtake
  pros::delay(5000);
  intake.move_voltage(-6000);//jitter to settle BALLS
  pros::delay(10);
  intake.move_voltage(12000);//Resume outtake
  pros::delay(2000);
  intake.move_voltage(0);//Stop outtake
  upperIntake.move_voltage(0);

  pros::delay(10000);//time pasued to manually push hood up
  intakeHood.retract();//Retract intake hood

  //Park robot (step 3)
  hood.extend();
  upperIntake.move_voltage(12000);//Hold any remaining BALLS
  intake.move_voltage(-12000);
  chassis.moveToPose(7-1, 73.43, 180, 3000);//Move near goal
  chassis.moveToPose(7-1, 110-20, 180, 3000, {.forwards = false, .maxSpeed = 60});//back away
  chassis.moveToPose(7-7, 50, 180, 3000, {.minSpeed = 120});//run into parking zone
  for(int i = 0; i < 100; i++){
    chassis.tank(-120, -120);
    pros::delay(100);
    chassis.tank(120, 120);
    pros::delay(1000);
  }
  /*
  //release BALLS (step 2 in sketch)
  chassis.moveToPose(48.26, 95.40, 90, 3000);//Move around goal
  chassis.moveToPose(95.41, 95.40, 90, 3000);//Move around goal
  chassis.moveToPose(123.57, 116.97, 90, 3000);//Get ready to back into goal
  chassis.moveToPose(92.5, 116.97, 90, 3000,{.forwards = false});//Back into goal 
  hood.extend();//Extend hood to prepare for outtake
  intake.move_voltage(12000);//Outtake BALLS into goal
  upperIntake.move_voltage(12000);//Outtake BALLS into goal
  intake.move_voltage(0);//Stop outtake
  upperIntake.move_voltage(0);
  
  pros::delay(10000000); //end code

  //Get into goal (step 3 in sketch)
  intakeHood.extend();//Extend intake hood to prepare for intake
  chassis.moveToPose(138.81-(23 + 3),//23 + x (higher x value = more push into goal)
                  23.44,//y value
                  90,//direction
                  3000//timeout
  );
  intake.move_voltage(12000);//Intake BALLS
  pros::delay(5000);//Wait to ensure all BALLS are intaken
  intake.move_voltage(0);//Stop intake
  intakeHood.retract();//Retract intake hood


  //Outtake balls into goal (step 4 in sketch)
  chassis.moveToPose(92,
                     23.44,
                     90,
                     3000
  );//Move into goal
  intake.move_voltage(12000);//Outtake BALLS into goal
  upperIntake.move_voltage(12000);//Outtake BALLS into goal
  pros::delay(5000);
  intake.move_voltage(0);//Stop outtake
  upperIntake.move_voltage(0);


  //reach other loader (step 5 in sketch)
  chassis.moveToPose(110, //x value
                     23.44,//y value
                     90,//direction
                     3000//timeout
  );//back out of goal
  chassis.moveToPose(100, //x value
                     70,//y value
                     -45,//direction
                     3000//timeout
  );//move into position to reach loader
  intakeHood.extend();//Extend intake hood to prepare for intake of BALLS
  chassis.moveToPose(138.81-(23 + 3),//23 + x (higher x value = more push into goal)
                     116.97,//y value
                     90,//direction
                     4000//timeout
  );//Move to the loader to intake BALLS
  intake.move_voltage(12000);
  pros::delay(5000);//Wait to ensure all BALLS are intaken
  intake.move_voltage(0);//Stop intake
  intakeHood.retract();//Retract intake hood

*/
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

      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) || master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      intake.move_voltage(12000);
      //upperIntake.move_voltage(5000);
      hood.retract();

    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) || master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      intake.move_voltage(12000);
      upperIntake.move_voltage(12000);
      hood.extend();
    
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
      intakeHood.extend();

    } else {
      intake.move_voltage(0);
      upperIntake.move_voltage(0);
      hood.retract();
      intakeHood.retract();
    }
	}
}
