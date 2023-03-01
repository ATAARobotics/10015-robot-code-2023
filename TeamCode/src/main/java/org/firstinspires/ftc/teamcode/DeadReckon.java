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

import java.util.ArrayList;


public class DeadReckon extends Object {
    // dead-reckoning
    public double pos_x;
    public double pos_y;
    public int last_slide;
    public int last_left;
    public int last_right;
    public ArrayList<Double> headings = new ArrayList<>();
    public double total_dist;

    public DeadReckon() {
        reset();
    }

    public void reset() {
        pos_x = 0.0;
        pos_y = 0.0;
        last_right = 0;
        last_left = 0;
        last_slide = 0;
        total_dist = 0;
    }

    public void update(DriveBase drive, Telemetry telemetry, double heading) {
        // XXX caller should tell this code "we will turn next time"
        // ... so short-circuit any imu integration and do the last
        // vector

        double head = 0.0;

        if (false) {
        // dead-reckoning
        // 1. collect an IMU heading, add to filter list
        headings.add(heading);
        if (headings.size() < 10) {
            return;
        }
        // XXX ooo, also we want to record "how long" we did each
        // angle .. so multiple by time here and then divide it all
        // out in the end?  no: we're averaging over the whole 5 samples

        // 2. find average heading
        head = 0.0;
        for (Double d : headings) {
            head += d;
        }
        head = head / headings.size();
        headings = new ArrayList<>();
        } else {
            head = heading;
        }

        telemetry.addData("dead_head", head);

        // 3. find the wheel closest to our heading ...that is, wheel
        // with less max angle offset from our (absolute!) heading

        // XXX wait, so these angles need to be differnet from what we
        // give hdrive -- if 0 is straight ahead, the WHEEL ANGLE of
        // slide is 90, the WHEEL ANGLE of right is 120,...
        double diff_left = 60 - heading;
        double diff_right = 300 - heading;
        double diff_slide = 180 - heading;

        // ...and actually, to help account for any rotation or other
        // weirdness we can always average the _closest two_ wheels to
        // our average heading.
        //
        // ...we could maybe be extra fancy and compute which one
        // contributes the most -- i.e. if we're trying to go "0"
        // exactly, and succeed, each of left + right is 15 degrees
        // away, so we take them equally. But if we're off a bit
        // (e.g. if we actually went "2.0" degrees) then we should
        // take _slightly_ more (15:15: versus 13:17) from one wheel
        // or the other (???)

        // 4. find _two_ closest wheels to our angle
        double min_diff = Math.min(diff_left, Math.min(diff_right, diff_slide));
        // ... so above will "prefer" right over left for 0 / forward

        telemetry.addData("dead_diff", min_diff);
        double ticks = 0.0;
        if (min_diff == diff_right) ticks = drive.motor_right.encoder.getPosition() - last_right;
        if (min_diff == diff_left) ticks = drive.motor_left.encoder.getPosition() - last_left;
        if (min_diff == diff_slide) ticks = drive.motor_slide.encoder.getPosition() - last_slide;
        last_right = drive.motor_right.encoder.getPosition();
        last_left = drive.motor_left.encoder.getPosition();
        last_slide = drive.motor_slide.encoder.getPosition();

        telemetry.addData("dead_ticks", ticks);

        // caliper says 87.5mm diameter, measured 294 ticks/rev
        // (could do ticks/rev at higher rev count than 1?)
        //double mm_per_tick = (Math.PI * 87.5) / 294.0;
        double mm_per_tick = (Math.PI * 90.0) / 294.0;
        double distance = ticks * mm_per_tick;

        // okay, so now we know that we've moved "distance" along the
        // "maximum wheel" direction .. and that the difference
        // between that and our actual direction is "min_diff"
        // degrees.

        double actual_distance = distance / Math.sin(Math.toRadians(min_diff));
        telemetry.addData("dead_actual", actual_distance);
        total_dist += actual_distance;
        telemetry.addData("dead_total", total_dist);

        // that "actual distance" above is along "heading" .. which is
        // in absolute / field co-ordinates already .. so we just need
        // the X + Y portions of it.

        double moved_x = actual_distance * Math.cos(Math.toRadians(head));
        double moved_y = actual_distance * Math.sin(Math.toRadians(head));

        // 3. convert to X and Y values
        pos_x += moved_x;//(moved_x * factor);
        pos_y += moved_y;//(moved_y * factor);

        telemetry.addData("pos_x", pos_x);
        telemetry.addData("pos_y", pos_y);
    }
}
