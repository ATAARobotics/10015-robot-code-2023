package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
@TeleOp(name="Oct29", group="TeleOp")

public class Oct29 extends LinearOpMode {
    private DemoRobotInterface robotui = null;
    public float axisX;
    public float axisY;

    public float boostOff = 0.2f;
    public float lowBoost = 0.4f;
    public float highBoost = 1f;

    public int lowPos = 1700;
    public int medPos = 2600;
    public int highPos = 6600;

    public int limitUP = 9000;
    public int limitDN = 0;

    public float speed = 0.75f;

    public boolean toggle = false;
    public boolean bool = toggle;
    public boolean val = false;

    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;



    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Mode", "starting...");
        telemetry.update();

        robotui = new DemoRobotInterface(hardwareMap, telemetry);

        robotui.clawS.setDirection(Servo.Direction.REVERSE);

        robotui.frm.setDirection(DcMotorSimple.Direction.REVERSE);
        robotui.flm.setDirection(DcMotorSimple.Direction.REVERSE);



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
            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("imu ", lastAngles.toString());
            telemetry.update();

            if(gamepad1.left_trigger > 0.5)
            {
                axisX = clamp(-gamepad1.left_stick_y, -boostOff, boostOff);
                axisY = clamp(-gamepad1.left_stick_x, -boostOff, boostOff);
            }
            else if(gamepad1.right_trigger > 0.5)
            {
                axisX = clamp(-gamepad1.left_stick_y, -highBoost, highBoost);
                axisY = clamp(-gamepad1.left_stick_x, -highBoost, highBoost);
            }
            else
            {
                axisX = clamp(-gamepad1.left_stick_y, -lowBoost, lowBoost);
                axisY = clamp(-gamepad1.left_stick_x, -lowBoost, lowBoost);
            }


            if((axisX <0.1)&&(axisX>-0.1)) // turn
            {
                robotui.flm.setPower(-axisY);
                robotui.frm.setPower(-axisY);
            }

            else if((axisY <0.1)&&(axisY>-0.1)) // go fwrd/bckwrds
            {
                robotui.flm.setPower(axisX);
                robotui.frm.setPower(axisX*-1);
            }


            if(gamepad2.y)
            {
                robotui.elevatorm.setTargetPosition(highPos);
                robotui.elevatorm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotui.elevatorm.setPower(speed);
            }

            if(gamepad2.b)
            {
                robotui.elevatorm.setTargetPosition(medPos);
                robotui.elevatorm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotui.elevatorm.setPower(speed);
            }

            if(gamepad2.a)
            {
                robotui.elevatorm.setTargetPosition(lowPos);
                robotui.elevatorm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotui.elevatorm.setPower(speed);
            }

            if(gamepad2.right_bumper)
            {
                robotui.elevatorm.setTargetPosition(limitDN);
                robotui.elevatorm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robotui.elevatorm.setPower(speed);
            }

            if(gamepad2.x)
            {
                robotui.elevatorm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if(robotui.elevatorm.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            {

                if(gamepad2.dpad_up && robotui.elevatorm.getCurrentPosition() < limitUP)
                {
                    robotui.elevatorm.setPower(speed);
                }
                else if(gamepad2.dpad_down && robotui.elevatorm.getCurrentPosition() > limitDN)
                {
                    robotui.elevatorm.setPower(-speed*2.5);
                }
                else
                {
                    robotui.elevatorm.setPower(0);
                }
            }

            if(robotui.elevatorm.getCurrentPosition() == robotui.elevatorm.getTargetPosition())
            {
                robotui.elevatorm.setPower(0);
            }


            if(gamepad2.left_trigger > 0.2)
            {
                robotui.clawF.setPosition(0.05);
                robotui.clawS.setPosition(0.05);
            }
            else if(gamepad2.right_trigger < 0.2)
            {
                robotui.clawF.setPosition(-0.1);
                robotui.clawS.setPosition(-0.1);
            }
        }
    }

    public static float clamp(final float value, final float min, final float max) {
        return (value < min) ? min : ((value > max) ? max : value);
    }
}
