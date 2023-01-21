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

@TeleOp(name="Kiwi: OpMode", group="Opmode")
public class KiwiDrive extends OpMode {
    // Declare OpMode motors.
    private Motor motor_left = null;
    private Motor motor_right = null;
    private Motor motor_slide = null;

    // Inertial Measurement Unit
    private IMU imu = null;

    private ColorSensor colour = null;

    // Holonomic Drive
    private HDrive drive = null;

    // claw + elevator setup
    private Motor motor_elevator = null;
    private Servo servo_claw_left = null;
    private Servo servo_claw_right = null;

    //servotoggle
    // false is open true is closed
    private boolean claw_position = false;

    //eleator position
    private int elevator_position = 0;

    // time-tracking
    private double last_time = 0.0;

    // controller-mode
    private int mode = 2;  // default

    // setup for various controls
    private GamepadEx gamepadex1 = null;
    private GamepadEx gamepadex2 = null;
    private ButtonReader bump_left = null;
    private ButtonReader bump_right = null;
    private ButtonReader button_a = null;
    private ButtonReader button_y = null;
    private ButtonReader button_b = null;
    private ButtonReader button_x = null;
    private ButtonReader dpad_down = null;
    private TriggerReader trigger_left = null;
    private TriggerReader trigger_right = null;
    private TriggerReader trigger_left2 = null;
    private TriggerReader trigger_right2 = null;

    @Override
    public void init() {
        telemetry.addData("status", "startup");
        telemetry.update();

        // initialize IMU
        IMU.Parameters imu_params = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            )
        );
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(imu_params);

        colour = hardwareMap.get(ColorSensor.class, "colour");

        // initialize motors
        motor_left = new Motor(hardwareMap, "left");
        motor_right = new Motor(hardwareMap, "right");
        motor_slide = new Motor(hardwareMap, "slide");
        motor_elevator = new Motor(hardwareMap, "hdelevator");

        motor_left.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_right.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_slide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_elevator.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        if (true) {
            motor_left.setRunMode(Motor.RunMode.VelocityControl);
            motor_right.setRunMode(Motor.RunMode.VelocityControl);
            motor_slide.setRunMode(Motor.RunMode.VelocityControl);
        } else {
            motor_left.setRunMode(Motor.RunMode.RawPower);
            motor_right.setRunMode(Motor.RunMode.RawPower);
            motor_slide.setRunMode(Motor.RunMode.RawPower);
        }
        motor_elevator.setRunMode(Motor.RunMode.PositionControl);
        motor_left.setInverted(false);
        motor_right.setInverted(false);
        motor_slide.setInverted(false);
        motor_elevator.setInverted(true); // so that "up" is positive for us
        motor_elevator.set(0);
        motor_elevator.setPositionCoefficient(0.02);

        // setup some controller listeners
        gamepadex1 = new GamepadEx(gamepad1);
        gamepadex2 = new GamepadEx(gamepad2);

        // elevator / claw motors
        servo_claw_left = hardwareMap.get(Servo.class, "hdclawleft");
        servo_claw_right = hardwareMap.get(Servo.class, "hdclawright");
        servo_claw_left.setDirection(Servo.Direction.FORWARD);
        servo_claw_right.setDirection(Servo.Direction.FORWARD);
        servo_claw_left.scaleRange(0.0, 1.0);
        servo_claw_right.scaleRange(0.0, 1.0);

        // initialize holonomic drive

        // first three arguments are the motors themselves, the next
        // three numbers are 'angles' for the motors -- this is their
        // mounting angle, relative to "0" being "forward"

        // NOTE NOTE!
        //    0. The angle "0" is straight ahead
        //    1. Angles are "right-hand coordinate" so "20" means "20 degress counter-clockwise"
        //    2. The motors ARE NOT IN counter-clockwise order! (you specify left, then right)
        //    3. Most angles are in RADIANS internally in ftclib (including these)
        drive = new HDrive(
            motor_left, motor_right, motor_slide,
            Math.toRadians(60), Math.toRadians(300), Math.toRadians(180)
        );
        drive.setMaxSpeed(0.50); // 0.0 to 1.0, percentage of "max"

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
        imu.resetYaw();
        motor_elevator.resetEncoder();
    }

    @Override
    public void loop() {
        // Executed repeatedly after a user presses Play (▶) but
        // before a user presses Stop (◼) on the Driver Station

        double diff = time - last_time;
        last_time = time;

        telemetry.addData("time", time);

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

        // let FTCLib updates it's button status
        gamepadex1.readButtons();
        gamepadex2.readButtons();


        // left / right BUMPERs switch mode
       // if (gamepadex1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            //mode -= 1;
       // } else if (gamepadex1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
        //     mode += 1;
        //}
        //if (mode < 0) mode = 2;
        //if (mode > 2) mode = 0;
        telemetry.addData("mode", mode);
        elevator_position = motor_elevator.getCurrentPosition();
        double elevator_speed = 0.65;

        // Elevator Controls (move to command?)
        double elevator_high_limit = 1575;
        double elevator_low_limit = 10;
        telemetry.addData("elevator-encoder", motor_elevator.getCurrentPosition());
        telemetry.addData("elevator-value", motor_elevator.get());
        telemetry.addData("elevator", "unknown");

        // elevator control

        //elevator presets
        if (false) {//gamepadex2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5 && gamepadex2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5) {
            //build-in P-controller presets
            //low 700 meduim 1190 high1560 could go higher

            motor_elevator.setRunMode(Motor.RunMode.PositionControl);

            if (gamepadex2.isDown(GamepadKeys.Button.A)) {
                motor_elevator.setTargetPosition(50);
                telemetry.addData("elevator", "preset-bottom");
            } else if (gamepadex2.isDown(GamepadKeys.Button.X)) {
                motor_elevator.setTargetPosition(700);
                telemetry.addData("elevator", "preset-short");
            } else if (gamepadex2.isDown(GamepadKeys.Button.B)) {
                motor_elevator.setTargetPosition(1190);
                telemetry.addData("elevator", "preset-medium");
            } else if (gamepadex2.isDown(GamepadKeys.Button.Y)) {
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
            double left_stick = gamepadex2.getLeftY() * 1.0;
//            if (gamepadex2.getLeftY() < 0.05 || gamepadex2.getLeftY() > 0.05) {
//                right_stick = gamepadex2.getLeftY() * 0.5;
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

        if (gamepadex2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >0.5 || gamepadex2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            motor_elevator.setTargetPosition(200);
            motor_elevator.setRunMode(Motor.RunMode.PositionControl);
            if (!motor_elevator.atTargetPosition()) {
                // XXX why does this ever successfuly go "down" at all??
                motor_elevator.set(0.1);
            }
        }
        //claw controls
        if (gamepadex2.wasJustPressed(GamepadKeys.Button.A)) {
            claw_position = !claw_position;
            if (claw_position == true) {  // closed
                servo_claw_left.setPosition(0.00);
                servo_claw_right.setPosition(1.00);
                //motor_elevator.set(0.5)for 1 second?
            } else { // must be false (open)
                servo_claw_left.setPosition(1.0);
                servo_claw_right.setPosition(0.0);
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

        // speed controls (percentage of max)
        double max_speed = 0.30;
        if (gamepadex1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5){
            // if left-trigger "pressed"
            max_speed = 0.40;
        } else if (gamepadex1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5){
            // if ONLY right-trigger "pressed"
            max_speed = 0.65;
        }
        drive.setMaxSpeed(max_speed);
        telemetry.addData("max_speed", max_speed);

        if (mode == 0) {
            // simple at first: left-strick forward/back + turn
            drive.driveRobotCentric(
                0.0, // strafe speed
                gamepad1.left_stick_y,  // forward/back (only) from left stick
                gamepad1.right_stick_x / 2.0 // turn from right stick, but less input
           );
        } else if (mode == 1) {
            drive.driveRobotCentric(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x / 2.0
            );
        } else if (mode == 2) {
            double heading = - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("heading", heading);

            drive.driveFieldCentric(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                // angelika likes backwards stick
                -gamepad1.right_stick_x * 0.75,
                heading
            );
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        // Executed once immediately after a user presses Stop (◼) on
        // the Driver Station
    }
}
