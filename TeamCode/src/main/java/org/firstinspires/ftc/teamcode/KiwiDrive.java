package org.firstinspires.ftc.teamcode;

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
    private ButtonReader bump_left = null;
    private ButtonReader bump_right = null;
    private ButtonReader button_a = null;

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

        motor_left.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_right.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor_slide.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        motor_left.setRunMode(Motor.RunMode.RawPower);
        motor_right.setRunMode(Motor.RunMode.RawPower);
        motor_slide.setRunMode(Motor.RunMode.RawPower);
        motor_left.setInverted(false);
        motor_right.setInverted(false);
        motor_slide.setInverted(false);

        // setup some controller listeners
        gamepadex1 = new GamepadEx(gamepad1);
        bump_left = new ButtonReader(gamepadex1, GamepadKeys.Button.LEFT_BUMPER);
        bump_right = new ButtonReader(gamepadex1, GamepadKeys.Button.RIGHT_BUMPER);
        button_a = new ButtonReader(gamepadex1, GamepadKeys.Button.A);

        //motor_elevator = hardwareMap.get(DcMotorEx.class, "elevator");

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

        // left / right BUMPERs switch mode
        if (bump_left.wasJustPressed()) {
            mode -= 1;
        } else if (bump_right.wasJustPressed()) {
            mode += 1;
        }
        if (mode < 0) mode = 2;
        if (mode > 2) mode = 0;
        telemetry.addData("mode", mode);

        // allow us to reset the yaw?
        if (button_a.wasJustPressed()) {
            imu.resetYaw();
        }

        if (mode == 0) {
            // simple at first: left-strick forward/back + turn
            drive.driveRobotCentric(
                0.0, // strafe speed
                -gamepad1.left_stick_y,  // forward/back (only) from left stick
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
