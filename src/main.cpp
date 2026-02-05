#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

// variable to select autonomous
int auton = 1;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motors
pros::MotorGroup Drivebase({1, -2, 3, -7, 5, -6}, pros::MotorGearset::blue); // all drivetrain motors
pros::MotorGroup leftMotors({1, -2, 3}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges
pros::MotorGroup rightMotors({-7, 5, -6}, pros::MotorGearset::blue); // right motors use 600 RPM cartridges
pros::MotorGroup intake({11}, pros::MotorGearset::blue); // All intake motors
pros::MotorGroup output({-10}, pros::MotorGearset::blue); // All output motors

pros::Motor l1(1, pros::MotorGearset::blue);
pros::Motor l2(-2, pros::MotorGearset::blue);
pros::Motor l3(3, pros::MotorGearset::blue);
pros::Motor r1(-7, pros::MotorGearset::blue);
pros::Motor r2(5, pros::MotorGearset::blue);
pros::Motor r3(-6, pros::MotorGearset::blue);

// Pneumatics
pros::adi::Pneumatics wing('a', false);

// Sensors
pros::Distance topDistance(16);
pros::Distance bottomDistance(17);
pros::Imu imu(18);


// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12, // 15 inch track width (for now)
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
							&imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
                        &throttle_curve,
                        &steer_curve
);


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
// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(true); // calibrate sensors
	pros::lcd::print(0, "Calibrating IMU...");
	while (imu.is_calibrating()){
		pros::delay(20);
	}
    //print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", imu.get_heading()); // heading
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


ASSET(basicLeft_txt); // Pure Pursuit path asset

void autonomous() {

    if (auton == 0){ // start on the left side
        pros::delay(200);
        intake.move(127);


        chassis.setPose(0, 0, 0);
        chassis.moveToPoint(0, 30, 1000);
        chassis.turnToHeading(-90, 1000);

        pros::delay(1000);
        chassis.moveToPoint(0, 34, 1000);
        chassis.moveToPose(0, 24, -90, 500);
        output.move(127);

        pros::delay(3000);
        chassis.moveToPoint(0, 30, 500);
        chassis.turnToHeading(135, 500);
        Drivebase.move(100);
        pros::delay(300);
        Drivebase.move(0);
        pros::delay(300);
        intake.move(-127);
        pros::delay(2000);
        intake.move(0);
        leftMotors.move(-60);
        pros::delay(200);
        Drivebase.move(0);

    }

	if (auton == 1){ // start on the right side
		chassis.setPose(0, 0, 0);
		intake.move(127);
		chassis.moveToPoint(0, 16,4000, {.maxSpeed=80});
		wing.extend();
		chassis.moveToPoint(0, 36, 4000, {.maxSpeed=40});
		while (topDistance.get_distance() > 110 || bottomDistance.get_distance() < 110){
			pros::delay(20);
		}
		pros::delay(1000);
		intake.move(0);
		chassis.turnToHeading(-135, 4000, {.maxSpeed=70, .earlyExitRange=0});
		chassis.moveToPoint(-30, 10, 4000, {.maxSpeed=60});
		chassis.turnToHeading(-170, 4000, {.earlyExitRange=0});
		chassis.moveToPoint(-30, 34, 4000, {.forwards=false, .maxSpeed=60});
		pros::delay(1500);
		intake.move(127);
		output.move(127);
		while (topDistance.get_distance() < 125 || bottomDistance.get_distance() < 125){
			pros::delay(20);
		}
		pros::delay(2000);
		intake.move(0);
		output.move(0);
		pros::delay(400);
	}

	if (auton == 2){ // skills
		chassis.setPose(0, 0, 0);
		intake.move(127);
		while (topDistance.get_distance() > 110 || bottomDistance.get_distance() < 110){
			pros::delay(20);
		}
		intake.move(0);
	}


    // if (auton == 1){ // start on the right side
    //  // Will be coded properly once robot is finished
    //  // Currently this is all previsonary code

    //  intake.move(127);

    //  // set chassis pose
    //  chassis.setPose(0, 0, 0);
    //  // lookahead distance: 15 inches
    //  // timeout: 2000 ms
    //  chassis.follow(basicLeft_txt, 15, 1000);
    //  intake.move(0);
    //  chassis.follow(movebacktohoopleft_txt, 15, 100);
    //  intake.move(127);

    //  pros::delay(3000);  
    //  intake.move(0);
    // }
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
	wing.extend();
   
    // loop forever
    while (true) {
		double heading = imu.get_heading();
		printf("Heading: %f\n", heading);
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        int deadzone = 10;
        if (abs(leftY) < deadzone) leftY = 0;
        if (abs(rightX) < deadzone) rightX = 0;
        chassis.arcade(leftY, -rightX, false, 0.75);



        static bool r1WasPressed;
        static bool r2WasPressed;
        static bool upWasPressed;
        static bool downWasPressed;
        static bool leftWasPressed;
        static bool rightWasPressed;
        static bool xWasPressed;
        static bool yWasPressed;
        static bool aWasPressed;
        static bool bWasPressed;
        static bool l1WasPressed;
       

        // Controller button states
        bool r1IsPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool r2IsPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool upIsPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
        bool downIsPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        bool leftIsPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
        bool rightIsPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
        bool xIsPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        bool yIsPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
        bool bIsPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        bool aIsPressed = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        bool l1IsPressed = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1);

        // Pneumatics control
        if (l1IsPressed){
            wing.toggle();
        }

        // intake & Hopper control
        if(r1IsPressed) {
            intake.move(127);
        } else if (r2IsPressed){
            intake.move(-127);
        }
        else if (upIsPressed){
            intake.move(127);
            output.move(127);
        }
        else{
            intake.move(0);
            output.move(0);
        }




        r1WasPressed = r1IsPressed;
        r2WasPressed = r2IsPressed;
        downWasPressed = downIsPressed;
        leftWasPressed = leftIsPressed;
        rightWasPressed = rightIsPressed;
        upWasPressed = upIsPressed;
        bWasPressed = bIsPressed;
        aWasPressed = aIsPressed;
        xWasPressed = xIsPressed;
        yWasPressed = yIsPressed;
        l1WasPressed = l1IsPressed;



        // delay to save resources
        pros::delay(20);
    }
}