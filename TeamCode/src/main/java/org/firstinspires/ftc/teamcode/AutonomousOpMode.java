/* Copyright (c) 100015
 *
 */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/mobilebasic
 */

@Autonomous(name="AutoOpMode", group="Linear Opmode")
public class AutonomousOpMode extends LinearOpMode {
    // Declare OpMode motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;
    private DcMotorEx elevatorDrive = null;
    private NormalizedColorSensor colorSensor = null;

    // declare servos
    private Servo clawServoL = null;
    private Servo clawServoR = null;
    private static final float mmPerInch        = 25.4f;
    // used for query information
    private long      e1, e2, e3, e4;
    private double    v1, v2, v3, v4;
    private double claw_value_left, claw_value_right;
    public String colorVal = "None";

    public final int twelveInches = 528;
    public final int sixInches = 264;
    public final int ninetyDegrees = 502;
    public final int fortyFiveDegrees = 251;
    public final float[] hsvValues = new float[3];

    // REV robotics ultra counts per revolution at motor = 28;
    // https://docs.revrobotics.com/duo-control/sensors/encoders/motor-based-encoders
    static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // 1120 or 560 20:1
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.8 ;     // For figuring circumference 90mm 3.543
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.25;
    static final double     TURN_SPEED              = 0.5;
    static final double     robotWidth = 6.63;

    static final int    elevator_min = 0;
    static final int    elevator_first = 10;
    static final int    elevator_second = 20;
    static final int    elevator_third = 30;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized Autonomous");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotorEx.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "rightDrive");
        elevatorDrive = hardwareMap.get(DcMotorEx.class, "elevatorDrive");
        elevatorDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorDrive.setDirection(DcMotor.Direction.FORWARD);
        elevatorDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //leftDrive.setTargetPositionTolerance();

        clawServoL = hardwareMap.get(Servo.class, "clawServoL");
        clawServoR = hardwareMap.get(Servo.class, "clawServoR");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        //leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        // The higher the value is, the faster the motor will move towards the target
        leftDrive.setPositionPIDFCoefficients(5.0);
        rightDrive.setPositionPIDFCoefficients(5.0);

        //leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        clawServoL.setDirection(Servo.Direction.FORWARD);
        clawServoR.setDirection(Servo.Direction.FORWARD);
        clawServoL.scaleRange(0.0,1.0);
        clawServoR.scaleRange( 0.0, 1.0);
        //clawServoL.setPosition(1.0);
        //clawServoR.setPosition(0.0);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        float colour_gain = 3;
        colorSensor.setGain(colour_gain);
        // Once per loop, we will update this hsvValues array. The first element (0) will contain the
        // hue, the second element (1) will contain the saturation, and the third element (2) will
        // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
        // for an explanation of HSV color.

        // Wait for the game to start (driver presses PLAY)
        elevator_level(elevator_min);
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double elevator_power;

            closeClaw();
            openClaw();
            // turn right 90
            moveDistance(-18);
            sleep(250);
            read_cone();
            sleep(250);
            moveDistance(-6);
            //sleep(500);
            if (colorVal == "Red"){
                rotateCW(90);
                moveDistance(-40);
            }
            else if (colorVal == "Blue"){
                //moveDistance(12);
                rotateCCW(90);
                moveDistance(-40);
            } else {
                moveDistance(-18);
                closeClaw();
            }

            // long is e %d
            // double is v %f
            e1 = leftDrive.getCurrentPosition();
            e2 = rightDrive.getCurrentPosition();
            e3 = elevatorDrive.getCurrentPosition();

            v1 = leftDrive.getVelocity();
            v2 = rightDrive.getVelocity();
            v3 = elevatorDrive.getVelocity();
            claw_value_left = clawServoL.getPosition();
            claw_value_right = clawServoR.getPosition();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "leftPower (%.2f), rightPower (%.2f)", leftPower, rightPower);
            telemetry.addData("Motors Pos", "leftPos (%d), rightPos (%d)", e1, e2);
            telemetry.addData("Motors Velocity", "leftV (%f), rightV (%f)", v1, v2);
            //telemetry.addData("Elevator", "elevatorPow (%.2f)", elevator_power);
            telemetry.addData("Elevator Pos", "Pos (%d)", e3);
            telemetry.addData("Elevator Velocity", "V (%f)", v3);
            telemetry.addData("Color", "name (%d)" + colorVal);
            telemetry.addData("ClawValue Auto", "LeftPos (%f) RightPos (%f)", claw_value_left, claw_value_right);
            telemetry.update();
            sleep(5000);
            break;

        }
    }

    public void closeClaw(){
        clawServoL.setPosition(0.0);
        clawServoR.setPosition(1.0);
        sleep(250);
    }

    public void openClaw(){
        clawServoL.setPosition(1.0);
        clawServoR.setPosition(0.0);
        sleep(250);
    }

    public void elevator_level (int level) {
        // set elevator to one foot (-253)
        int currentPos = elevatorDrive.getCurrentPosition();
        telemetry.addData("elevaotr pos", currentPos);
        telemetry.update();

        elevatorDrive.setTargetPosition((int) level); // -253
        elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorDrive.setVelocity(800);
        while (elevatorDrive.isBusy()){
            sleep(100);
            int currentPoser = elevatorDrive.getCurrentPosition();
            telemetry.addData("elevaotr pos", currentPoser);
            telemetry.update();
        }
        sleep(250);
    }

    public void moveDistance(double distance){
        long initialL = leftDrive.getCurrentPosition();
        long initialR = rightDrive.getCurrentPosition();
        double destL = initialL + distance*COUNTS_PER_INCH;
        double destR = initialR + distance*COUNTS_PER_INCH;

        leftDrive.setTargetPosition((int) destL);
        rightDrive.setTargetPosition((int) destR);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //leftDrive.setPower(DRIVE_SPEED);
        //rightDrive.setPower(DRIVE_SPEED);
        leftDrive.setVelocity(800);
        rightDrive.setVelocity(800);

        while (leftDrive.isBusy() && rightDrive.isBusy() ){
            sleep(100);
        }

        //leftDrive.setPower(0);
        //rightDrive.setPower(0);

        sleep(250);   // optional pause after each move.
    }

    public void rotateCW(int degrees){
        long initialL = leftDrive.getCurrentPosition()+degrees;
        long initialR = rightDrive.getCurrentPosition()-degrees;

        leftDrive.setTargetPosition((int) initialL);
        rightDrive.setTargetPosition((int) initialR);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(0.60);
        rightDrive.setPower(0.60);

        while (leftDrive.isBusy() || rightDrive.isBusy() ){
            sleep(100);
        }

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
    }
    public void rotateCCW(int degrees){
        // rotate left
        long initialL = leftDrive.getCurrentPosition()-degrees;
        long initialR = rightDrive.getCurrentPosition()+degrees;

        leftDrive.setTargetPosition((int) initialL);
        rightDrive.setTargetPosition((int) initialR);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(0.60);
        rightDrive.setPower(0.60);

        while (leftDrive.isBusy() || rightDrive.isBusy() ){
            sleep(100);
        }

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
    }

    public void read_cone(){
        // get sensor values + normalise using Color class
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        float max = Math.max(colors.red, colors.blue);
        float maxA = Math.max(max, colors.green);
        if (maxA >0.002f){
            if (colors.red == maxA){
                colorVal ="Red";
            } else if (colors.green == maxA) {
                colorVal ="Green";
            } else {
                colorVal ="Blue";
            }
        }
    }
}
