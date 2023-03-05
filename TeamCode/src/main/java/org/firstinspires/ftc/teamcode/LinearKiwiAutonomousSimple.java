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
import org.opencv.core.Mat;
import org.opencv.objdetect.QRCodeDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import org.openftc.apriltag.AprilTagDetection;

import org.firstinspires.ftc.teamcode.LinearKiwiAutonomous;


@Autonomous(name="Kiwi: Simple Autonomous (RIGHT)", group="Autonomous")
public class LinearKiwiAutonomousSimple extends LinearKiwiAutonomous {

    // to traverse "one tile" is XXX seconds? 2.5?

    public void add_todo_list_post_detection(List<Action> todo, double field_factor, int code_number) {
        todo.add(new DriveAction(-0.5, 0, 0.0, 2.38 * field_factor)); // west

        // XXX 94.0cm is what we need for the strafe distance
        todo.add(new DriveDistanceAction(0.0, -0.5, -90, -910.0)); // north 93.5cm (okay 91.0cm?)

        todo.add(new ElevatorAction(1740, 0.15)); //go to high position
        todo.add(new DriveHeadingAction(-0.6, 0, 2.50 * field_factor, -90.0)); // west
        // theoretically, we've slammed the pole -- back up a little
        todo.add(new DriveAction(0.5, 0, 0.0, 0.3 * field_factor)); // west
        todo.add(new ElevatorAction(1460, 0.05)); // down a little
        todo.add(new ClawAction()); //open
        todo.add(new DriveAction(0.5, 0, 0.0, 0.80 * field_factor)); // east
        todo.add(new SlowElevatorDownAction(400)); //go to drive position

        todo.add(new DriveDistanceAction(0.0, -0.5, -90, -234.0)); // north

        /*

        todo.add(new DriveAction(-0.5, 0.0, 0.0, 2.1 * field_factor)); // west
        todo.add(new DriveAction(0.0, -0.5, 0.0, 0.57 * field_factor)); // north
        todo.add(new ElevatorAction(1660, 0.15)); //go to high position
        todo.add(new DriveAction(-0.5, 0.0, 0.0, 0.50 * field_factor));//left
        todo.add(new ElevatorAction(1460, 0.05)); // down a little
        todo.add(new ClawAction()); //open
        todo.add(new DriveAction(0.5,0.0,0,0.8 * field_factor));
        todo.add(new SlowElevatorDownAction(400)); //go to drive position
        todo.add(new DriveAction(0.0, -0.5, 0.0, 1.0 * field_factor)); // north
        */

        if (code_number == 1) {
            // already there
            //todo.add(new DriveAction(0.5, 0.0, 0.0, 0.4 * field_factor));
        } else if (code_number == 3) {
            //todo.add(new DriveAction(0.8, 0.0, 0.0, 3.8 * field_factor));
            // turbo?
            todo.add(new DriveAction(0.8, 0.0, 0.0, 3.00 * field_factor));
        } else {
            todo.add(new DriveAction(0.5, 0, 0.0, 2.5 * field_factor));
        }
        todo.add(new SlowElevatorDownAction(20)); // "bottom" basically
    }
}
