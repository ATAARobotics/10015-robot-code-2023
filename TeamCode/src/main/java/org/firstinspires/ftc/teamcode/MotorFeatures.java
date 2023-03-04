package org.firstinspires.ftc.teamcode;

import java.util.List;
import java.util.ArrayList;

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


@Autonomous(name="Motor Features", group="Autonomous")
public class MotorFeatures extends LinearOpMode {

    private DriveBase drivebase = null;
    private Elevator elevator = null;

    private void ensure_stop(double heading) {
        /// ideally shouldn't need this, but .. here we are
        drivebase.drive.driveFieldCentric(0.0, 0.0, 0.0, heading);
        drivebase.motor_left.set(0.0);
        drivebase.motor_right.set(0.0);
        drivebase.motor_slide.set(0.0);
    }

    public class Action {
        public double start_time = -1.0;

        public void do_drive(double heading, DriveBase drivebase) {
            drivebase.drive.driveFieldCentric(0.0, 0.0, 0.0, heading);
        }

        public void do_elevator(Elevator elevator) {
            if (!elevator.motor_elevator.atTargetPosition()) {
                elevator.motor_elevator.set(0.15);
            }
        }

        public boolean is_done(double time) {
            return false;
        }
    }

    public class DriveAction extends Action {
        public double stick_x = 0.0;
        public double stick_y = 0.0;
        public double turn = 0.0;
        public double duration = 0.0;

        public DriveAction(double x, double y, double t, double dur) {
            stick_x = x;
            stick_y = y;
            turn = t;
            duration = dur;
        }

        public void do_drive(double heading, DriveBase drivebase) {
            drivebase.drive.driveFieldCentric(stick_x, stick_y, turn, heading);
        }

        public boolean is_done(double time) {
            if ((time - start_time) > duration) {
                return true;
            }
            return false;
        }
    }

    @Override
    public void runOpMode() {
        //
        // setup
        //

        telemetry.addData("status", "startup");
        telemetry.update();

        // let the drivebase set itself up
        drivebase = new DriveBase(hardwareMap);

        // elevator setup
        elevator = new Elevator(hardwareMap);

        drivebase.motor_left.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        drivebase.motor_right.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        drivebase.motor_slide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        drivebase.motor_left.resetEncoder();
        drivebase.motor_right.resetEncoder();
        drivebase.motor_slide.resetEncoder();

        telemetry.addData("status", "initialized");
        telemetry.update();

        //
        // done setup wait for start
        //

        waitForStart();

        //
        // start has been pressed
        //

        // left  0 = 4, turn CW 1 rev: -289     (-293)
        // slide 1 = -185, turn CW 1 rev: -479  (-293)
        // right 2 = 22, turn CW 1 rev: 273     (-294?)

        // make sure robot starts at correct position
        drivebase.imu.resetYaw();
        // ensure we have "zero" at the bottom of our elevator
        elevator.motor_elevator.resetEncoder();
        elevator.motor_elevator.setRunMode(Motor.RunMode.PositionControl);

        double heading = 0.0;
        double left_start = time;
        double right_start = time;
        double slide_start = time;

        drivebase.motor_left.setRunMode(Motor.RunMode.RawPower);
        drivebase.motor_right.setRunMode(Motor.RunMode.RawPower);
        drivebase.motor_slide.setRunMode(Motor.RunMode.RawPower);

        // different motors achieving different speeds here
        // sometimes jumping to huge number...
        // left 2540

        double start_ticks = 0;
        double start_time = 0;
        boolean started = false;

        // okay, looks like left achieves 505 rpms
        // (540 with only one motor)
        // 2660 or 2640 from encoder.getRawVelocity()... (with one motor only)

        while (opModeIsActive()) {
            heading = - drivebase.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            // approach full power in 3 seconds
            drivebase.motor_left.set(Math.min(1.0, (time - left_start) / 3.0));
            /*
            drivebase.motor_right.set(Math.min(1.0, (time - right_start) / 3.0));
            drivebase.motor_slide.set(Math.min(1.0, (time - slide_start) / 3.0));
            */

            if (time > 4.0 && !started) {
                started = true;
                start_time = time;
                start_ticks = drivebase.motor_left.encoder.getPosition();
            }

            double left_rpm = 0.0;

            if (started && time > left_start) {
                double ticks = drivebase.motor_left.encoder.getPosition() - start_ticks;
                left_rpm = (ticks / 294.0) / ((time - start_time) / 60.0);
            }

            telemetry.addData("time", time);
            telemetry.addData("heading", heading);
            telemetry.addData(
                "left",
                String.format(
                    "position=%.2f encoder=%d rpm=%.2f",
                    drivebase.motor_left.encoder.getRawVelocity(),
                    drivebase.motor_left.encoder.getPosition(),
                    left_rpm
                )
            );
            telemetry.addData(
                "right",
                String.format(
                    "position=%.2f encoder=%d",
                    drivebase.motor_right.getCorrectedVelocity(),
                    drivebase.motor_right.encoder.getPosition()
                )
            );
            telemetry.addData(
                "slide",
                String.format(
                    "position=%.2f encoder=%d",
                    drivebase.motor_slide.getCorrectedVelocity(),
                    drivebase.motor_slide.encoder.getPosition()
                )
            );
            telemetry.update();
        }
    }
}