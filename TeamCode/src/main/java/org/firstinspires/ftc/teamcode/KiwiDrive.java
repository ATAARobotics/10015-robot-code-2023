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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

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


@TeleOp(name="Kiwi: TeleOp", group="Opmode")
public class KiwiDrive extends OpMode {

    private DriveBase drivebase = null;
    private Elevator elevator = null;

    private GamepadEx gamepadex1 = null;
    private GamepadEx gamepadex2 = null;

    // time-tracking
    private double last_time = 0.0;

    // our own velocity tracking
    private double last_left = 0;
    private double last_right = 0;
    private double last_slide = 0;

    private FtcDashboard dashboard;

    @Override
    public void init() {
        // override the dashboard with FTC dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("status", "startup");
        telemetry.update();

        // setup some controller listeners
        gamepadex1 = new GamepadEx(gamepad1);
        gamepadex2 = new GamepadEx(gamepad2);

        // let the drivebase set itself up
        drivebase = new DriveBase(hardwareMap);

        // elevator setup
        elevator = new Elevator(hardwareMap);

        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Executed repeatedly after a user presses INIT but before a
        // user presses Play (▶) on the Driver Station
    }

    @Override
    public void start() {
        // Executed once immediately after a user presses Play (▶) on
        // the Driver Station

        // XXX VITAL: for competition, the AUTONOMOUS mode code will do these
        // reset calls -- DO NOT do them again here...

        // but for practice, we leave it alone..

        if (gamepadex1.isDown(GamepadKeys.Button.A) || gamepadex1.isDown(GamepadKeys.Button.X)) {
            // make sure robot starts at correct position
            drivebase.imu.resetYaw();
            // ensure we have "zero" at the bottom of our elevator
            elevator.motor_elevator.resetEncoder();
        }
    }

    @Override
    public void loop() {
        // Executed repeatedly after a user presses Play (▶) but
        // before a user presses Stop (◼) on the Driver Station

        double diff = time - last_time;
        last_time = time;

        telemetry.addData("time", time);

        // let FTCLib updates its button status
        // VERY IMPORTANT: only do this _once_ per loop (e.g. not in
        // do_drive_updates()) because otherwise the notion of
        // "wasJustPressed" is wrong
        gamepadex1.readButtons();
        gamepadex2.readButtons();

        // tie controllers through to components
        boolean slow_override = false;

        //toggle the slow overide
        if (elevator.motor_elevator.getCurrentPosition() > 800) {
            slow_override = true;
        }
        drivebase.do_drive_updates(gamepadex1, telemetry, slow_override);
        elevator.do_elevator_updates(gamepadex2, telemetry);

        // our own velocity tracking
        double vel_left = (drivebase.motor_left.encoder.getPosition() - last_left) / diff;
        double vel_right = (drivebase.motor_right.encoder.getPosition() - last_right) / diff;
        double vel_slide = (drivebase.motor_slide.encoder.getPosition() - last_slide) / diff;
        last_left = drivebase.motor_left.encoder.getPosition();
        last_right = drivebase.motor_right.encoder.getPosition();
        last_slide = drivebase.motor_slide.encoder.getPosition();
        telemetry.addData("vel_left", vel_left);
        telemetry.addData("vel_right", vel_right);
        telemetry.addData("vel_slide", vel_slide);

        // for the FTC dashboard
        telemetry.addData(
            "left",
            drivebase.motor_left.encoder.getRawVelocity()
        );
        telemetry.addData(
            "right",
            drivebase.motor_right.encoder.getRawVelocity()
        );
        telemetry.addData(
            "slide",
            drivebase.motor_slide.encoder.getRawVelocity()
        );

        // can draw, if we have dead-reckoning code again
        TelemetryPacket p = new TelemetryPacket();
        // we Strongly Suspect the field is in "inches" in the dashboard... hence "25.4"
        // p.fieldOverlay()
        //     .setStrokeWidth(1)
        //     .setStroke("red")
        //     .setFill("black")
        //     .fillCircle(drivebase.dead.pos_x / 25.4, drivebase.dead.pos_y / 25.4, 5);
        // dashboard.sendTelemetryPacket(p);

        telemetry.update();
    }

    @Override
    public void stop() {
        // Executed once immediately after a user presses Stop (◼) on
        // the Driver Station
    }
}
