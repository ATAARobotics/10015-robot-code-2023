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

import org.firstinspires.ftc.teamcode.LinearKiwiAutonomous;





@Autonomous(name="Kiwi: Linear Autonomous (LEFT)", group="Autonomous")
public class LinearKiwiAutonomousLeft extends LinearKiwiAutonomous {

    public void add_todo_list_post_detection(List<Action> todo, double field_factor, int code_number) {
        todo.add(new DriveAction(0.0, -0.5, 0.0, 1.5 * field_factor)); // north
        todo.add(new WaitAction(1.0));
        todo.add(new DriveAction(-0.5, 0.0, 0.0, 2.0 * field_factor));
        todo.add(new DriveAction(0.0, -0.5, 0.0, 0.85 * field_factor)); // north
        todo.add(new ElevatorAction(1650)); //go to high position
        todo.add(new DriveAction(-0.5, 0.0, 0.0, 0.9 * field_factor));//left
        todo.add(new ClawAction()); //open
        todo.add(new DriveAction(0.5,0.0,0,0.8 * field_factor));
        todo.add(new ElevatorAction(300)); //go to drive position
        todo.add(new DriveAction(0.0, -0.5, 0.0, 1.0 * field_factor)); // north
        //todo.add(new OtherTurnAction(0.2, 189.0));  // turn around (so we drive forwards)

        if (code_number == 1) {
            // already there
        } else if (code_number == 2) {
            todo.add(new DriveAction(0.5, 0, 0.0, 2.0 * field_factor));
        } else {
            //todo.add(new DriveAction(0.8, 0.0, 0.0, 3.8 * field_factor));
            // turbo?
            todo.add(new DriveAction(0.7, 0.0, 0.0, 2.5 * field_factor));
        }
    }

}
