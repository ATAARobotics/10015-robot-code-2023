package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

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


@TeleOp(name="Kiwi: OpMode", group="Opmode")
public class KiwiDrive extends OpMode {
    // Declare OpMode motors.
    private Motor motor_left = null;
    private Motor motor_right = null;
    private Motor motor_slide = null;

    private Motor motor_elevator = null;
    //private NormalizedColorSensor colorSensor = null;

    // Inertial Measurement Unit
    private IMU imu = null;

    // Holonomic Drive
    private HDrive drive = null;

    // servos (for the claw)
    private Servo servo_claw_left = null;
    private Servo servo_claw_right = null;

    // time-tracking
    private double last_time = 0.0;

    // controller-mode
    private int mode = 0;  // default

    // setup for various controls
    private GamepadEx gamepadex1 = null;
    private GamepadEx gamepadex2 = null;
    private ButtonReader bump_left = null;
    private ButtonReader bump_right = null;
    private ButtonReader button_a = null;
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

        // initialize motors
        motor_left = new Motor(hardwareMap, "left");
        motor_right = new Motor(hardwareMap, "right");
        motor_slide = new Motor(hardwareMap, "slide");
        motor_elevator = new Motor(hardwareMap, "hdelevator");

        motor_left.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_right.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_slide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_elevator.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        motor_left.setRunMode(Motor.RunMode.RawPower);
        motor_right.setRunMode(Motor.RunMode.RawPower);
        motor_slide.setRunMode(Motor.RunMode.RawPower);
        motor_left.setInverted(false);
        motor_right.setInverted(false);
        motor_slide.setInverted(false);

        // setup some controller listeners
        gamepadex1 = new GamepadEx(gamepad1);
        gamepadex2 = new GamepadEx(gamepad2);

        motor_elevator = new Motor(hardwareMap, "hdelevator");

        //servo_claw_left = hardwareMap.get(Servo.class, "clawLeft");
        //servo_claw_right = hardwareMap.get(Servo.class, "clawRight");

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

        // let FTCLib updates it's button status
        gamepadex1.readButtons();
        gamepadex2.readButtons();

        // Elevator Controls (move to command?)
        telemetry.addData("elevator-encoder", motor_elevator.getCurrentPosition());
        telemetry.addData("elevator", "unknown");
        if (gamepadex2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            //motor_elevator.set(0.5);
            telemetry.addData("elevator","up");
        }
        else if (gamepadex2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            //motor_elevator.set(-0.5);
            telemetry.addData("elevator","down");
        }
        else {
            //motor_elevator.set(0);
            telemetry.addData("elevator","stop");
        }

        // left / right BUMPERs switch mode
        if (gamepadex1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            mode -= 1;
        } else if (gamepadex1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            mode += 1;
        }
        if (mode < 0) mode = 2;
        if (mode > 2) mode = 0;
        telemetry.addData("mode", mode);

        // allow us to reset the yaw?
        if (gamepadex1.wasJustPressed(GamepadKeys.Button.A)) {
            imu.resetYaw();
        }
        // speed controls (percentage of max)
        double max_speed = 0.45;
        if (gamepadex1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5){
            // if left-trigger "pressed"
            max_speed = 0.30;
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
                gamepad1.right_stick_x * 0.75,
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
