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

import org.firstinspires.ftc.teamcode.DriveBase;

public class Elevator extends Object {

    // claw + elevator setup
    public Motor motor_elevator = null;
    public Servo servo_claw_left = null;
    public Servo servo_claw_right = null;

    //servotoggle
    // false is open true is closed
    public boolean claw_position = false;

    //eleator position
    public int elevator_position = 0;
    //are we holding a position?
    private boolean hold_position = false;
    private int hold_at_position = 0;

    public Elevator(HardwareMap hardwareMap) {
        // initialize motors
        motor_elevator = new Motor(hardwareMap, "hdelevator");
        motor_elevator.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_elevator.setRunMode(Motor.RunMode.PositionControl);
        motor_elevator.setInverted(true); // so that "up" is positive for us
        motor_elevator.set(0);
        motor_elevator.setPositionCoefficient(0.02);

        // elevator / claw motors
        servo_claw_left = hardwareMap.get(Servo.class, "hdclawleft");
        servo_claw_right = hardwareMap.get(Servo.class, "hdclawright");
        servo_claw_left.setDirection(Servo.Direction.FORWARD);
        servo_claw_right.setDirection(Servo.Direction.FORWARD);
        servo_claw_left.scaleRange(0.0, 1.0);
        servo_claw_right.scaleRange(0.0, 1.0);
    }

    public void close_claw() {
        servo_claw_left.setPosition(0.00);
        servo_claw_right.setPosition(1.00);
    }
    public void open_claw() {
        servo_claw_left.setPosition(1.0);
        servo_claw_right.setPosition(0.0);
    }
    public void do_elevator_updates(GamepadEx gamepadex, Telemetry telemetry) {
        // Elevator Controls
        elevator_position = motor_elevator.getCurrentPosition();
        double elevator_speed = 0.65;

        double elevator_high_limit = 1575;
        double elevator_low_limit = 10;
        telemetry.addData("elevator-encoder", motor_elevator.getCurrentPosition());
        telemetry.addData("elevator-value", motor_elevator.get());
        telemetry.addData("elevator", "unknown");

        // elevator control

        //elevator presets
        if (false) {//gamepadex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5 && gamepadex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5) {
            //build-in P-controller presets
            //low 700 meduim 1190 high1560 could go higher

            motor_elevator.setRunMode(Motor.RunMode.PositionControl);

            if (gamepadex.isDown(GamepadKeys.Button.A)) {
                motor_elevator.setTargetPosition(50);
                telemetry.addData("elevator", "preset-bottom");
            } else if (gamepadex.isDown(GamepadKeys.Button.X)) {
                motor_elevator.setTargetPosition(700);
                telemetry.addData("elevator", "preset-short");
            } else if (gamepadex.isDown(GamepadKeys.Button.B)) {
                motor_elevator.setTargetPosition(1190);
                telemetry.addData("elevator", "preset-medium");
            } else if (gamepadex.isDown(GamepadKeys.Button.Y)) {
                motor_elevator.setTargetPosition(1550);
                telemetry.addData("elevator", "preset-tall");
            } else {
                motor_elevator.set(0);
                telemetry.addData("elevator","stop");
            }

            // actually input the control to the target
            if (!motor_elevator.atTargetPosition()) {
                // XXX why does this ever successfuly go "down" at all??
                motor_elevator.set(0.15);
            }
        } else {
            // manual elevator control
            // setting the speed to0.05 counteracts the pull of gravity
            motor_elevator.setRunMode(Motor.RunMode.RawPower);
            double left_stick = gamepadex.getLeftY() * 1.0;
//            if (gamepadex.getLeftY() < 0.05 || gamepadex.getLeftY() > 0.05) {
//                right_stick = gamepadex.getLeftY() * 0.5;
//            }
            if (left_stick > 0.05) {
                motor_elevator.setRunMode(Motor.RunMode.RawPower);
                if (motor_elevator.getCurrentPosition() < elevator_high_limit) {
                    motor_elevator.set(elevator_speed * left_stick);
                    telemetry.addData("elevator", "up");
                }
            } else if (left_stick < -0.05) {
                motor_elevator.setRunMode(Motor.RunMode.RawPower);
                if (motor_elevator.getCurrentPosition() > elevator_low_limit) {
                    // go really slow if we're close to the bottom
                    if (motor_elevator.getCurrentPosition() < 400) {
                        motor_elevator.set(elevator_speed * left_stick * 0.2);
                    } else {
                        motor_elevator.set(elevator_speed * left_stick * 0.2);
                    }
                    telemetry.addData("elevator","down");
                }
            } else {
                motor_elevator.setRunMode(Motor.RunMode.RawPower);
                motor_elevator.set(0.0);
                telemetry.addData("elevator","stop");
            }
        }

        // XXX note to self: charleigh request: right trigger is "hold
        // at current position", left trigger is "hold at preset
        // height of 380"

        if (gamepadex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >0.5 ){
            motor_elevator.setTargetPosition(400);
            motor_elevator.setRunMode(Motor.RunMode.PositionControl);
            if (!motor_elevator.atTargetPosition()) {
                // XXX why does this ever successfuly go "down" at all??
                motor_elevator.set(0.1);
            }
        }
        if (gamepadex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5){
            if (!hold_position) {
                hold_position = true;
                hold_at_position = elevator_position;
            }
            if (hold_position == true) {
                motor_elevator.setTargetPosition(hold_at_position);
                motor_elevator.setRunMode(Motor.RunMode.PositionControl);
                if (!motor_elevator.atTargetPosition()) {
                    motor_elevator.set(0.15);
                }
            }
        }
        else {
            hold_position = false;
        }
        //claw controls
        if (gamepadex.wasJustPressed(GamepadKeys.Button.A) || gamepadex.wasJustPressed(GamepadKeys.Button.X)) {
            claw_position = !claw_position;
            if (claw_position == true) {  // closed
                telemetry.addData("claw position", "open");
                close_claw();
                //motor_elevator.set(0.5)for 1 second?
            } else { // must be false (open)
                telemetry.addData("claw position", "closed");
                open_claw();
            }
        }
        telemetry.addData(
            "servos",
            String.format(
                "claw %b left %.2f right %.2f",
                claw_position,
                servo_claw_left.getPosition(),
                servo_claw_right.getPosition()
            )
        );
    }
}
