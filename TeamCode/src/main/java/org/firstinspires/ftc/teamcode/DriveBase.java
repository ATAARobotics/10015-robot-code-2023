package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor;

import com.qualcomm.hardware.bosch.BNO055IMU;

// this is just a dumb wrapper
//import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.controller.PIDController;





//
// this abstracts out the drive-related robot functions and motors to
// be used by the KiwiDrive opmode and autonomous opmode
//
public class DriveBase extends Object {
    // Kiwi Drive motors
    public Motor motor_left = null;
    public Motor motor_right = null;
    public Motor motor_slide = null;

    // Inertial Measurement Unit
    public IMU imu = null;

    // Holonomic Drive
    public HDrive drive = null;

    // controller-mode
    private int mode = 2;  // default

    public DriveBase(HardwareMap hardwareMap) {
        // initialize IMU
        IMU.Parameters imu_params = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            )
        );
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(imu_params);

        // initialize motors
        motor_left = new Motor(hardwareMap, "left");
        motor_right = new Motor(hardwareMap, "right");
        motor_slide = new Motor(hardwareMap, "slide");

        motor_left.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_right.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_slide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        motor_left.setRunMode(Motor.RunMode.VelocityControl);
        motor_right.setRunMode(Motor.RunMode.VelocityControl);
        motor_slide.setRunMode(Motor.RunMode.VelocityControl);

        motor_left.setInverted(false);
        motor_right.setInverted(false);
        motor_slide.setInverted(false);

        // initialize holonomic drive

        // first three arguments are the motors themselves, the next
        // three numbers are 'angles' for the motors -- this is their
        // mounting angle, relative to "0" being "forward"

        // NOTE NOTE!
        //    0. The angle "0" is straight ahead
        //    1. Angles are "right-hand coordinate" so "20" means "20 degress counter-clockwise"
        //    2. The motors ARE NOT IN counter-clockwise order! (you specify left, then right)
        //    3. Most angles are in RADIANS internally in ftclib (including these)
        drive = new HDrive(
            motor_left, motor_right, motor_slide,
            Math.toRadians(60), Math.toRadians(300), Math.toRadians(180)
        );
        drive.setMaxSpeed(0.50); // 0.0 to 1.0, percentage of "max"
    }

    // call this repeatedly from the OpMode.loop() function to "do"
    // the drive stuff from the given controller
    public void do_drive_updates(GamepadEx gamepadex, Telemetry telemetry) {
        // let FTCLib updates its button status (probably done in parent, but ...)
        gamepadex.readButtons();
        telemetry.addData("mode", mode);

        // speed controls (percentage of max)
        double max_speed = 0.37;
        if (gamepadex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5){
            // if left-trigger "pressed"
            max_speed = 0.42;
        } else if (gamepadex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5){
            // if ONLY right-trigger "pressed"
            max_speed = 0.65;
        }
        drive.setMaxSpeed(max_speed);
        telemetry.addData("max_speed", max_speed);

        if (mode == 0) {
            // simple at first: left-strick forward/back + turn
            drive.driveRobotCentric(
                0.0, // strafe speed
                gamepadex.gamepad.left_stick_y,  // forward/back (only) from left stick
                gamepadex.gamepad.right_stick_x / 2.0 // turn from right stick, but less input
           );
        } else if (mode == 1) {
            drive.driveRobotCentric(
                gamepadex.gamepad.left_stick_x,
                gamepadex.gamepad.left_stick_y,
                gamepadex.gamepad.right_stick_x / 2.0
            );
        } else if (mode == 2) {
            double heading = - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("heading", heading);

            drive.driveFieldCentric(
                gamepadex.gamepad.left_stick_x,
                gamepadex.gamepad.left_stick_y,
                // angelika likes backwards stick
                -gamepadex.gamepad.right_stick_x * 0.75,
                heading
            );
        }
    }
}
