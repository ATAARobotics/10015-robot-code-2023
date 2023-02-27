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


public class DeadReckon extends Object {
    // dead-reckoning
    public double pos_x;
    public double pos_y;
    public int last_slide;
    public int last_left;
    public int last_right;
    public ArrayList<Double> headings = new ArrayList<>();

    public DeadReckon() {
        reset();
    }

    public void reset() {
        pos_x = 0.0;
        pos_y = 0.0;
        last_right = 0;
        last_left = 0;
        last_slide = 0;
    }

    public void update(DriveBase drive, Telemetry telemetry) {
        // XXX caller should tell this code "we will turn next time"
        // ... so short-circuit any imu integration and do the last
        // vector

        // dead-reckoning
        // 1. collect an IMU heading, add to filter list
        headings.add(new Double(drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
        if (headings.size() < 5) {
            return;
        }

        // 2. find average heading
        double head = 0.0;
        for (Double d : headings) {
            head += d;
        }
        head = head / headings.size();
        headings.removeAll();

        // 3. find the wheel closest to our robot-centric heading
        // ...that is, wheel with less max angle offset to 0
        double diff_right = heading - 60.0;
        double diff_slide = heading - 180.0;
        double diff_left = heading - 300.0;
        double min_diff = Math.min(diff_left, Math.min(diff_right, diff_slide));

        double ticks = 0.0;
        if (min_diff == diff_right) ticks = drive.motor_right.encoder.getPosition();
        if (min_diff == diff_left) ticks = drive.motor_left.encoder.getPosition();
        if (min_diff == diff_slide) ticks = drive.motor_slide.encoder.getPosition();

        // caliber says 87.5mm diameter, measured 294 ticks/rev
        // (could do ticks/rev at higher rev count than 1?)
        double mm_per_tick = (Math.PI * 87.5) / 294.0;
        double distance = ticks * mm_per_tick;

        // okay, so now we know that we've moved "distance" along the
        // "maximum wheel" direction .. and that the difference
        // between that and our actual direction is "min_diff"
        // degrees.

        double actual_distance = distance / Math.sin(Math.toRadians(min_diff));

        // that "actual distance" above is along "heading" .. which is
        // in absolute / field co-ordinates already .. so we just need
        // the X + Y portions of it.

        double moved_x = actual_distance * Math.cos(head);
        double moved_y = actual_distance * Math.sin(head);

        // 3. convert to X and Y values
        pos_x += moved_x;//(moved_x * factor);
        pos_y += moved_y;//(moved_y * factor);

        telemetry.addData("pos_x", pos_x);
        telemetry.addData("pos_y", pos_y);
    }

}
