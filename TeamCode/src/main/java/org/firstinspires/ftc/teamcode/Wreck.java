package org.firstinspires.ftc.teamcode;

import java.util.List;
import java.util.ArrayList;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.controller.PIDController;
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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

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
import org.opencv.core.Mat;
import org.opencv.objdetect.QRCodeDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import org.openftc.apriltag.AprilTagDetection;

//import org.firstinspires.ftc.teamcode.OpenCv.FindQrCodePipeline;


@Autonomous(name="Wreck", group="Autonomous")
public class Wreck extends LinearOpMode {

    private DriveBase drivebase = null;
    private Elevator elevator = null;

    private OpenCvCamera camera = null;
    public int the_code = -1;

    private FtcDashboard dashboard;

    private void ensure_stop(double heading) {
        /// ideally shouldn't need this, but .. here we are
        drivebase.drive.driveFieldCentric(0.0, 0.0, 0.0, heading);
        drivebase.motor_left.set(0.0);
        drivebase.motor_right.set(0.0);
        drivebase.motor_slide.set(0.0);
    }

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //
        // setup
        //

        telemetry.addData("status", "startup");
        telemetry.update();

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

        // wait half a second
        sleep(500);

        double heading;
        double start = time;
        double stick_y = -0.5;

        List<Double> headings = new ArrayList<Double>();
        double start_left;
        double start_right;
        double start_slide;

        // 0.04 is 1/25 .. that is, expecting at most a 25-degree
        // error ..  in reality this could be as high as 180 at most,
        // worst case
        PIDController heading_control = new PIDController(0.005, 0.0001, 0.0);
        double desired_heading = 0.0; // want to go exactly forward

        double accum_heading = 0.0;
        int heading_count = 0;

        double total_distance = 0.0;
        drivebase.reset();

        while (opModeIsActive()) {
            heading = - drivebase.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            accum_heading += heading;
            heading_count += 1;

            // looks like we're out by 5% on longer drives?

            // average over 4 "ticks"
            if (heading_count > 4) {
                heading = accum_heading / heading_count;
                accum_heading = 0.0;
                heading_count = 0;

                telemetry.addData("stick_y", stick_y);
                double turn = heading_control.calculate(heading, desired_heading);
                telemetry.addData("pid_turn", turn);
                // PID turn output is pretty aggressive
                //drivebase.drive.driveFieldCentric(0.0, stick_y, turn / 5.0, heading);
                drivebase.drive.driveFieldCentric(0.0, stick_y, 0.0, heading);
                drivebase.dead.update(drivebase, telemetry, heading);
                telemetry.addData("enc_left", drivebase.motor_left.encoder.getPosition());
                telemetry.addData("enc_right", drivebase.motor_right.encoder.getPosition());
                telemetry.addData("enc_slide", drivebase.motor_slide.encoder.getPosition());
                //if (total_distance < 300.0 && time - start <= 2.0) {
                if (time - start <= 3.0) {
                    headings.add(heading);
                    total_distance = distanceTravelled(
                        headings,
                        drivebase.motor_left.encoder.getPosition(),
                        drivebase.motor_right.encoder.getPosition(),
                        drivebase.motor_slide.encoder.getPosition()
                        );
                    telemetry.addData("d", total_distance);
                } else {
                    stick_y = 0.0;
                }
                telemetry.update();
            }
        }
    }

    public double distanceTravelled(List<Double> headings, double left, double right, double slide) {
        double mm_per_tick = (Math.PI * 87.5) / 294.0;
        double avg_head = 0.0;
        for (Double d : headings) {
            avg_head += d;
        }
        avg_head = avg_head / headings.size();
        telemetry.addData("avg_head", avg_head);

        double left_distance = (left * mm_per_tick) / Math.sin(Math.toRadians(60 - avg_head));
        double right_distance = (right * mm_per_tick) / Math.sin(Math.toRadians(300 - avg_head));
        double slide_distance = (slide * mm_per_tick) / Math.sin(Math.toRadians(180 - avg_head));

        telemetry.addData("travel_left", left_distance);
        telemetry.addData("travel_right", right_distance);
        telemetry.addData("travel_slide", slide_distance);

        double dist = Math.min(left_distance, right_distance);
        telemetry.addData("travel_avg", dist);

        double travel_no_slide = (((left * mm_per_tick) + (right * mm_per_tick)) * 4.0) / 3.0;
        telemetry.addData("messing_around", travel_no_slide);

        double dx = dist * Math.cos(Math.toRadians(avg_head));
        double dy = dist * Math.sin(Math.toRadians(avg_head));
        telemetry.addData("moved_x", dx);
        telemetry.addData("moved_y", dy);

        return dist;
    }
}

