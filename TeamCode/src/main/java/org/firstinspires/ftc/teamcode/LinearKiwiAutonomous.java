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


@Autonomous(name="Kiwi: Linear Autonomous", group="Autonomous")
public class LinearKiwiAutonomous extends LinearOpMode {

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

    public class TurnAction extends Action {
        public double stick_x = 0.0;
        public double stick_y = 0.0;
        public double turn = 0.0;
        public double target_heading = 0.0;
        public boolean done = false;

        public TurnAction(double t, double target) {
            stick_x = 0.0;
            stick_y = 0.0;
            turn = t;
            target_heading = target;
        }

        public void do_drive(double heading, DriveBase drivebase) {
            drivebase.drive.driveFieldCentric(stick_x, stick_y, turn, heading);
            if (heading < target_heading) {
                done = true;
            }
        }

        public boolean is_done(double time) {
            return done;
        }
    }

    public class DetectColourAction extends Action {
        public double duration = 0.0;
        public LinearKiwiAutonomous auto = null;
        int r = 0;
        int g = 0;
        int b = 0;

        public DetectColourAction(LinearKiwiAutonomous a, double d) {
            duration = d;
            auto = a;
        }

        public void do_drive(double heading, DriveBase drivebase) {
            drivebase.drive.driveFieldCentric(0.0, 0.0, 0.0, heading);
            r += auto.colour.red();
            g += auto.colour.green();
            b += auto.colour.blue();
        }

        public boolean is_done(double time) {
            if ((time - start_time) > duration) {
                int max = Math.max(r, Math.max(g, b));
                String detected_colour = "unknown";
                if (r == max) detected_colour = "red";
                if (g == max) detected_colour = "green";
                if (b == max) detected_colour = "blue";
                auto.found_colour = detected_colour;
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
        elevator.open_claw();
        sleep(1000);
        elevator.close_claw();
        sleep(1500);
        elevator.motor_elevator.setRunMode(Motor.RunMode.PositionControl);
        elevator.motor_elevator.setTargetPosition(300);

        //
        // main logic loop
        //

        List<Action> todo = new ArrayList<Action>();

        double heading = 0.0;

        // drive ahead, slowly, for a little while
        todo.add(new DriveAction(0.0, -0.5, 0.0, 1.0)); // ahead
        todo.add(new TurnAction(-0.2, -89.0));  // turn to line up sensor
        todo.add(new DriveAction(-0.4, 0.0, 0.0, 0.30)); // strafe a bit
        todo.add(new DriveAction(0.0, -0.40, 0.0, 2.3));  // drive to cone
        todo.add(new DetectColourAction(this, 1.6));

        while (!todo.isEmpty() && opModeIsActive()) {
            telemetry.addData("todo", todo.size());
            Action doing = todo.get(0);
            todo.remove(doing);
            doing.start_time = time;
            while (!doing.is_done(time) && opModeIsActive()) {
                heading = - drivebase.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                doing.do_drive(heading, drivebase);
                doing.do_elevator(elevator);
                telemetry.addData("start_time", doing.start_time);
                telemetry.addData("time", time);
                telemetry.addData("heading", heading);
                telemetry.update();
            }
        }

        telemetry.addData("colour", found_colour);
        telemetry.update();
        if (found_colour == "green") {
            todo.add(new DriveAction(0.0, -0.5, 0.0, 0.2));
        } else if (found_colour == "red") {
            todo.add(new DriveAction(-0.5, 0.0, 0.0, 3.0));
        } else { // blue
            todo.add(new DriveAction(0.5, -0.5, 0.0, 0.4));
            todo.add(new DriveAction(0.5, 0.0, 0.0, 2.5));
        }

        while (!todo.isEmpty() && opModeIsActive()) {
            Action doing = todo.get(0);
            todo.remove(doing);
            doing.start_time = time;
            while (!doing.is_done(time)) {
                heading = - drivebase.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                doing.do_drive(heading, drivebase);
            }
        }

        // done
        ensure_stop(heading);
        elevator.motor_elevator.setTargetPosition(10);
        double timeout = time + 2.0;
        while (!elevator.motor_elevator.atTargetPosition() && opModeIsActive() && time < timeout) {
            // XXX why does this ever successfuly go "down" at all??
            elevator.motor_elevator.set(0.15);
        }
        elevator.motor_elevator.setRunMode(Motor.RunMode.RawPower);
        sleep(1500);
    }
}
