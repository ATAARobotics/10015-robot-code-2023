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


@TeleOp(name="Kiwi: OpMode", group="Opmode")
public class KiwiDrive extends OpMode {

    private DriveBase drivebase = null;
    private Elevator elevator = null;

    private GamepadEx gamepadex1 = null;
    private GamepadEx gamepadex2 = null;

    private ColorSensor colour = null;

    // time-tracking
    private double last_time = 0.0;

    @Override
    public void init() {
        telemetry.addData("status", "startup");
        telemetry.update();

        //colour = hardwareMap.get(ColorSensor.class, "colour");

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

        // make sure robot starts at correct position
        drivebase.imu.resetYaw();
        // ensure we have "zero" at the bottom of our elevator
        elevator.motor_elevator.resetEncoder();
    }

    @Override
    public void loop() {
        // Executed repeatedly after a user presses Play (▶) but
        // before a user presses Stop (◼) on the Driver Station

        double diff = time - last_time;
        last_time = time;

        telemetry.addData("time", time);

        if (false) {
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
        }

        // let FTCLib updates its button status
        // VERY IMPORTANT: only do this _once_ per loop (e.g. not in
        // do_drive_updates()) because otherwise the notion of
        // "wasJustPressed" is wrong
        gamepadex1.readButtons();
        gamepadex2.readButtons();

        // tie controllers through to components
        drivebase.do_drive_updates(gamepadex1, telemetry);
        elevator.do_elevator_updates(gamepadex2, telemetry);

        telemetry.update();
    }

    @Override
    public void stop() {
        // Executed once immediately after a user presses Stop (◼) on
        // the Driver Station
    }
}
