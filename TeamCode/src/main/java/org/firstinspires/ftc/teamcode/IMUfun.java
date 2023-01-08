package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// this is just a dumb wrapper
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

@TeleOp(name="kiwi", group="TeleOp")
public class IMUfun extends LinearOpMode {
//    private DemoRobotInterface robotui = null;
  //  RevIMU               imu;
    BNO055IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Mode", "starting...");
        telemetry.update();


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // initialize our motor + servo variables from the hardwareMap
    //    Motor motor_left = hardwareMap.get(DcMotorEx.class, "left");
      //  Motor motor_right = hardwareMap.get(DcMotorEx.class, "right");
        //Motor motor_slide = hardwareMap.get(DcMotorEx.class, "slide");
        Motor motor_left = new Motor(hardwareMap, "left");
        Motor motor_right = new Motor(hardwareMap, "right");
        Motor motor_slide = new Motor(hardwareMap, "slide");
        //motor_elevator = hardwareMap.get(DcMotorEx.class, "elevator");

        //servo_claw_left = hardwareMap.get(Servo.class, "clawLeft");
        //servo_claw_right = hardwareMap.get(Servo.class, "clawRight");

        HDrive kiwi = new HDrive(motor_left, motor_right, motor_slide);
        GamepadEx driverOp = new GamepadEx(gamepad1);

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();


        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        while (opModeIsActive())
        {
            //lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //telemetry.addData("the IMU ", lastAngles.toString());
            telemetry.update();
            kiwi.driveRobotCentric(driverOp.getLeftX(), driverOp.getLeftY(), driverOp.getRightY());
        }
    }
}
