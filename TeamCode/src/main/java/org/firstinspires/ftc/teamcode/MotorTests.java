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


@Autonomous(name="Motor Tests", group="Autonomous")
public class MotorTests extends LinearOpMode {

    private DriveBase drivebase = null;
    private Elevator elevator = null;

    private ColorSensor colour = null;
    private DistanceSensor distance = null;
    private String found_colour = "unknown";


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
        elevator.motor_elevator.setRunMode(Motor.RunMode.PositionControl);

        double heading = 0.0;
        double left_start = 0.0;
        double right_start = 0.0;
        double slide_start = 0.0;

        while (opModeIsActive()) {
            heading = - drivebase.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            if (gamepadex1.isDown(GamepadKeys.Button.A)) {
                if (gamepadex1.wasJustPressed(GamepadKeys.Button.A)) {
                    left_start = time;
                }
                drivebase.motor_left.setRunMode(Motor.RunMode.RawPower);
                // approach full power in 3 seconds
                drivebase.motor_left.set(Math.min(1.0, (time - left_start) / 3.0));
            } else {
                drivebase.motor_left.setRunMode(Motor.RunMode.VelocityControl);
                drivebase.motor_left.set(0.0);
            }

            if (gamepadex1.isDown(GamepadKeys.Button.B)) {
                if (gamepadex1.wasJustPressed(GamepadKeys.Button.B)) {
                    left_start = time;
                }
                drivebase.motor_right.setRunMode(Motor.RunMode.RawPower);
                // approach full power in 3 seconds
                drivebase.motor_right.set(Math.min(1.0, (time - right_start) / 3.0));
            } else {
                drivebase.motor_right.setRunMode(Motor.RunMode.VelocityControl);
                drivebase.motor_right.set(0.0);
            }

            if (gamepadex1.isDown(GamepadKeys.Button.X)) {
                if (gamepadex1.wasJustPressed(GamepadKeys.Button.X)) {
                    slide_start = time;
                }
                drivebase.motor_slide.setRunMode(Motor.RunMode.RawPower);
                // approach full power in 3 seconds
                drivebase.motor_slide.set(Math.min(1.0, (time - slide_start) / 3.0));
            } else {
                drivebase.motor_slide.setRunMode(Motor.RunMode.VelocityControl);
                drivebase.motor_slide.set(0.0);
            }

            telemetry.addData("time", time);
            telemetry.addData("heading", heading);
            telemetry.addData(
                "colour",
                String.format("r=%.2f g=%.2f b=%.2f", colour.red(), colour.green(), colour.blue())
            );
            telemetry.addData(
                "left",
                String.format(
                    "position=%.2f encoder=%.2f",
                    drivebase.motor_left.getCurrentPosition(),
                    drivebase.motor_left.encoder.getPosition()
                )
            );
            telemetry.addData(
                "right",
                String.format(
                    "position=%.2f encoder=%.2f",
                    drivebase.motor_left.getCurrentPosition(),
                    drivebase.motor_left.encoder.getPosition()
                )
            );
            telemetry.addData(
                "slide",
                String.format(
                    "position=%.2f encoder=%.2f",
                    drivebase.motor_left.getCurrentPosition(),
                    drivebase.motor_left.encoder.getPosition()
                )
            );
            telemetry.update();
        }
    }
}
