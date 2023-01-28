package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

// our other classes, for 10015
import org.firstinspires.ftc.teamcode.DriveBase;
import org.firstinspires.ftc.teamcode.Elevator;


@Autonomous(name="Kiwi: Linear Autonomous", group="Autonomous")
public class LinearKiwiAutonomous extends LinearOpMode {

    private DriveBase drivebase = null;
    private Elevator elevator = null;

    private ColorSensor colour = null;
    private DistanceSensor distance = null;
    private String found_colour = "unknown";

    private String update_colour() {
        String detected_colour = "unknown";
        int r = colour.red();
        int g = colour.green();
        int b = colour.blue();
        int max = Math.max(colour.alpha(), Math.max(b, Math.max(r, g)));
        if (r == max) { detected_colour = "red"; }
        if (g == max) { detected_colour = "green"; }
        if (b == max) { detected_colour = "blue"; }
        telemetry.addData(
            "colour",
            String.format("colour: red=%d green=%d blue=%d max=%d detected=%s", r, g, b, max, detected_colour)
        );
        return detected_colour;
    }

    private void ensure_stop(double heading) {
        /// ideally shouldn't need this, but .. here we are
        drivebase.drive.driveFieldCentric(0.0, 0.0, 0.0, heading);
        drivebase.motor_left.set(0.0);
        drivebase.motor_right.set(0.0);
        drivebase.motor_slide.set(0.0);
    }


    @Override
    public void runOpMode() {
        //
        // setup
        //

        telemetry.addData("status", "startup");
        telemetry.update();

        colour = hardwareMap.get(ColorSensor.class, "colour");
        colour.enableLed(true);

        distance = hardwareMap.get(DistanceSensor.class, "colour");

        // let the drivebase set itself up
        drivebase = new DriveBase(hardwareMap);

        // elevator setup
        elevator = new Elevator(hardwareMap);

        telemetry.addData("status", "initialized");
        telemetry.update();

        //
        // done setup wait for start
        //

        waitForStart();

        //
        // start has been pressed
        //

        // make sure robot starts at correct position
        drivebase.imu.resetYaw();
        // ensure we have "zero" at the bottom of our elevator
        elevator.motor_elevator.resetEncoder();
        elevator.close_claw();

        sleep(500);

        //
        // main logic loop
        //

        double heading = 0.0;

        // drive ahead, slowly, for a little while
        int iterations = 20;
        while (iterations > 0) {
            heading = - drivebase.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            drivebase.drive.driveFieldCentric(
                0.0,
                -0.5,
                0.0,
                heading
            );
            sleep(25);
            --iterations;
        }

        // turn until we're about 90 degrees
        while (heading > -89.0) {
            heading = - drivebase.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            drivebase.drive.driveFieldCentric(
                0.0,
                0.0,
                -0.2,
                heading
            );
            sleep(25);
            --iterations;
        }

        // strafe "left" a little
        iterations = 9;
        while (iterations > 0) {
            heading = - drivebase.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            drivebase.drive.driveFieldCentric(
                -0.3,
                0.0,
                0.0,
                heading
            );
            sleep(25);
            --iterations;
        }

        // drive towards the cone
        iterations = 30;
        while (iterations > 0) {
            heading = - drivebase.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            drivebase.drive.driveFieldCentric(
                0.0,
                -0.40,
                0.0,
                heading
            );
            --iterations;
            sleep(20);
        }
        ensure_stop(heading);

        // "take in the colours" for a while (filter them)
        iterations = 80;
        int red = 0;
        int green = 0;
        int blue = 0;
        while (iterations > 0) {
            red += colour.red();
            green += colour.green();
            blue += colour.blue();

            heading = - drivebase.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            drivebase.drive.driveFieldCentric(
                0.0,
                0.0,
                0.0,
                heading
            );

            sleep(20);
            --iterations;
        }

        int max = Math.max(red, Math.max(green, blue));
        String detected_colour = "unknown";
        if (red == max) detected_colour = "red";
        if (green == max) detected_colour = "green";
        if (blue == max) detected_colour = "blue";
        telemetry.addData("detected", detected_colour);
        telemetry.update();

        // now, do something based on the colour:
        // green: drive forward a little
        // red/blue: drive "left" / "right"

        double stick_x = 0.0;
        double stick_y = 0.0;
        if (detected_colour == "green") {
            iterations = 20;
            stick_y = -0.5;
        } else {
            iterations = 90; // red or blue
            if (detected_colour == "red") {
                stick_x = -0.5;
            } else {
                // must be blue
                stick_x = 0.5;
            }
        }

        // XXX actually, probably better to loop in these "as fast as
        // possible" and use global time as the limit? (more feedback
        // to the controller)
        while (iterations > 0) {
            --iterations;
            heading = - drivebase.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            drivebase.drive.driveFieldCentric(
                stick_x,
                stick_y,
                0.0,
                heading
            );
            sleep(25);
        }

        // done
        ensure_stop(heading);
    }
}
