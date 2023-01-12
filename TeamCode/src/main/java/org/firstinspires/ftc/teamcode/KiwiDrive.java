package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.hardware.bosch.BNO055IMU;

// this is just a dumb wrapper
//import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.gamepad.GamepadEx;


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

    // declare servos
    private Servo servo_claw_left = null;
    private Servo servo_claw_right = null;

    // time-tracking
    private double last_time = 0.0;

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

        // motor power levels
        motor_left.setRunMode(Motor.RunMode.RawPower);
        motor_right.setRunMode(Motor.RunMode.RawPower);
        motor_slide.setRunMode(Motor.RunMode.RawPower);
        motor_left.setInverted(false);
        motor_right.setInverted(false);
        motor_slide.setInverted(false);


        //motor_elevator = hardwareMap.get(DcMotorEx.class, "elevator");

        //servo_claw_left = hardwareMap.get(Servo.class, "clawLeft");
        //servo_claw_right = hardwareMap.get(Servo.class, "clawRight");

        // initialize holonomic drive
        // first three are the motors, next three numbers are 'angles' for the motors
        drive = new HDrive(
            motor_left, motor_right, motor_slide,
            60, 120, 270
        );

        // motor-specific setups

        // elevator setup
        // https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotor.ZeroPowerBehavior.html
        //motor_elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motor_elevator.setDirection(DcMotor.Direction.FORWARD);

        // claw servo setup
        //servo_claw_left.setDirection(Servo.Direction.FORWARD);
        //servo_claw_right.setDirection(Servo.Direction.FORWARD);
        // XXX what do these mean?
        //servo_claw_left.scaleRange(0.0, 1.0);
        //servo_claw_right.scaleRange( 0.0, 1.0);

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

        // XXX reset variables, etc here
    }

    @Override
    public void loop() {
        // Executed repeatedly after a user presses Play (▶) but
        // before a user presses Stop (◼) on the Driver Station

        double diff = time - last_time;
        last_time = time;

        telemetry.addData("time", time);
        telemetry.addData("diff", diff);
        telemetry.addData(
            "stick",
            String.format(
                "stick: left (%.2f, %.2f) right (%.2f, %.2f)",
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                gamepad1.right_stick_y
            )
        );
        telemetry.update();

        // simple at first: left-strick forward/back + turn
        drive.driveRobotCentric(0.0, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }

    @Override
    public void stop() {
        // Executed once immediately after a user presses Stop (◼) on
        // the Driver Station
    }
}
