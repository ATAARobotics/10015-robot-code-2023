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
        // dead-reckoning
        // 1. how far has each wheel spun? (in TICKs!)
        double left = drive.motor_left.encoder.getPosition() - last_left;
        double right = drive.motor_right.encoder.getPosition() - last_right;
        double slide = drive.motor_slide.encoder.getPosition() - last_slide;
        last_left = drive.motor_left.encoder.getPosition();
        last_right = drive.motor_right.encoder.getPosition();
        last_slide = drive.motor_slide.encoder.getPosition();

        // XXX allegedly the wheel is 90mm accross
        // TODO: caliper our exact wheels at buildspace
        double mm_per_tick = (2.0 * Math.PI * 43) / 294.0;

        // 2. convert movements to mm
        left = left * mm_per_tick;
        right = right * mm_per_tick;
        slide = slide * mm_per_tick;

        double theta = Math.toRadians(30);
        // 2. apply rotation matrix, yielding x, y values for motion
        double moved_y = (-left * Math.cos(theta)) + (right * Math.cos(theta));
        double moved_x = (left * Math.sin(theta)) + (right * Math.sin(theta)) - slide;

        // this is wrong, so we try to adjust by some "empirically computed" factor
        // ...or the math is just fucked
        double factor = 0.6159760225629736;

        // 3. convert to X and Y values
        pos_x += (moved_x * factor);
        pos_y += (moved_y * factor);

        telemetry.addData("pos_x", pos_x);
        telemetry.addData("pos_y", pos_y);
    }

}
