#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pose.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMotors({
    11,
    12,
    13,
});
pros::MotorGroup rightMotors({-18, -19, -20});
pros::Imu imu(17);
pros::Rotation horizontalEnc(15, true);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -3.7);

lemlib::Drivetrain drivetrain(
    &leftMotors, // left motor group
    &rightMotors, // right motor group
    9.75, // 10 inch track width
    2.75, // using new 2.75" omnis
    600, // drivetrain rpm is 360
    2 // chase power is 2. If we had traction wheels, it would have been 8
);
lemlib::ControllerSettings linearController(
    10, // proportional gain (kP)
    0, // integral gain (kI)
    3, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
);
lemlib::ControllerSettings angularController(
    2, // proportional gain (kP)
    0, // integral gain (kI)
    10, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);
lemlib::OdomSensors sensors(
    nullptr, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
    &imu // inertial sensor
);
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        lemlib::Pose pose(0, 0, 0);
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

void disabled() {}

void competition_initialize() {}

pros::Motor intake(3);

ASSET(test_txt)

void autonomous() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveToPose(0, 24, 0, 5000);
    chassis.turnTo(-24, 60, 5000);
    intake = 127;
    chassis.moveToPose(-24, 60, -45, 5000, {.maxSpeed = 127, .minSpeed = 100}, false);
    chassis.moveToPose(-24, 60, -90, 500, {.maxSpeed = 127, .minSpeed = 100}, false);
    chassis.moveToPose(0, 60, -90, 5000, {.forwards = false, .maxSpeed = 127, .minSpeed = 100}, false);
}

void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.curvature(leftY, rightX);
        // delay to save resources
        pros::delay(10);
    }
}
