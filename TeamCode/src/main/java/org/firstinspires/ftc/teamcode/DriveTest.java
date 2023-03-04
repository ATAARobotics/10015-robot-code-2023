package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

import java.util.ArrayList;


@Autonomous(name="Drive Test", group="Opmode")
public class DriveTest extends OpMode {

    private DriveBase drivebase = null;
    private Elevator elevator = null;

    // time-tracking
    private double last_time = 0.0;
    ArrayList<Double> times = new ArrayList<>();

    private FtcDashboard dashboard;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("status", "startup");
        telemetry.update();

        // let the drivebase set itself up
        drivebase = new DriveBase(hardwareMap);
        drivebase.reset();

        // elevator setup
        elevator = new Elevator(hardwareMap);

        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Executed repeatedly after a user presses INIT but before a
        // user presses Play (▶) on the Driver Station
        telemetry.addData("vel_left", drivebase.motor_left.encoder.getRawVelocity());
        telemetry.addData("vel_right", drivebase.motor_right.encoder.getRawVelocity());
        telemetry.addData("vel_slide", drivebase.motor_slide.encoder.getRawVelocity());
    }

    @Override
    public void start() {
        // Executed once immediately after a user presses Play (▶) on
        // the Driver Station

        // make sure robot starts at correct position
        drivebase.imu.resetYaw();
        // ensure we have "zero" at the bottom of our elevator
        elevator.motor_elevator.resetEncoder();
        drivebase.dead.reset();
    }

    @Override
    public void loop() {
        // Executed repeatedly after a user presses Play (▶) but
        // before a user presses Stop (◼) on the Driver Station

        double diff = time - last_time;

        // running-average of our cycle-time
        times.add(new Double(diff));
        if (times.size() > 5) {
            times.remove(0);
        }
        double avg_diff = 0.0;
        for (Double d : times) {
            avg_diff += d;
        }
        avg_diff = avg_diff / times.size();
        last_time = time;
        telemetry.addData("time", time);

        double heading = - drivebase.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData("heading", heading);
        drivebase.dead.update(drivebase, telemetry);

        if (time >= 15.0 && time < 17.0) {
            drivebase.drive.driveFieldCentric(0, -1.0, 0, heading);
        } else {
            drivebase.drive.driveFieldCentric(0, 0, 0, heading);
        }

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

        TelemetryPacket p = new TelemetryPacket();
        // we Strongly Suspect the field is in "inches" in the dashboard... hence "25.4"
        //
        double mm_per_tick = (2.0 * Math.PI * 43) / 294.0;
        p.fieldOverlay()
            .setStrokeWidth(1)
            .setStroke("red")
            .setFill("black")
            .fillCircle((drivebase.dead.pos_x * mm_per_tick) / 25.4, (drivebase.dead.pos_y * mm_per_tick) / 25.4, 5);
        dashboard.sendTelemetryPacket(p);

        // raw encoders
        telemetry.addData("encoder_left", drivebase.motor_left.encoder.getPosition());
        telemetry.addData("encoder_right", drivebase.motor_right.encoder.getPosition());
        telemetry.addData("encoder_slide", drivebase.motor_slide.encoder.getPosition());

        // max when doing -1.0 input to drive forward seems to be about 600
        // so actual-max should be bigger?
        telemetry.addData("vel_left", drivebase.motor_left.encoder.getRawVelocity());
        telemetry.addData("vel_right", drivebase.motor_right.encoder.getRawVelocity());
        telemetry.addData("vel_slide", drivebase.motor_slide.encoder.getRawVelocity());

        telemetry.addData("diff", diff);
        telemetry.addData("diff_avg", avg_diff);

        telemetry.update();
    }

    @Override
    public void stop() {
        // Executed once immediately after a user presses Stop (◼) on
        // the Driver Station
    }
}
