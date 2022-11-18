/* Copyright (c) 10015
 *
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 *
 */


public class ElevatorUtils {
    // Declare OpMode motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;
    public DcMotorEx elevatorDrive = null;

    // declare servos
    private Servo clawServoL = null;
    private Servo clawServoR = null;

    // used for query information
    private long      e1, e2, e3, e4;
    private double    v1, v2, v3, v4, claw_value_left, claw_value_right;


    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotorEx.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "rightDrive");
        elevatorDrive = hardwareMap.get(DcMotorEx.class, "elevatorDrive");
        //https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/DcMotor.ZeroPowerBehavior.html
        elevatorDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawServoL = hardwareMap.get(Servo.class, "clawServoL");
        clawServoR = hardwareMap.get(Servo.class, "clawServoR");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // record default run modes of motors
        //originalRunMode = leftDrive.getMode();
        //arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //originalRightMode = rightDrive.getMode();

        clawServoL.setDirection(Servo.Direction.FORWARD);
        clawServoR.setDirection(Servo.Direction.REVERSE);
        clawServoL.scaleRange(0.0,1.0);

        //clawServoR.scaleRange( 0.0, 1.0);
        clawServoL.setPosition(0.18);
        clawServoR.setPosition(0.8);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // query MotorEx positions
            // Uses 1 bulk-read to obtain ALL the motor data

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double elevator_power;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_y;
            double elevator_up = gamepad1.left_trigger;
            double elevator_down = -gamepad1.right_trigger;
            boolean claw = gamepad1.cross;
            boolean unclaw = gamepad1.square;
            boolean moveForward = gamepad1.circle;

            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            elevator_power = Range.clip(elevator_down+elevator_up, -1.0, 1.0);

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            //leftPower  = -gamepad1.left_stick_y ;
            //rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower/2);
            rightDrive.setPower(rightPower/2);
            elevatorDrive.setPower(elevator_power);

            // claw on
            if (claw) {
                clawServoL.setPosition(0.0);
                clawServoR.setPosition(0.5);
            }
            // claw off
            if (unclaw){
                clawServoL.setPosition(0.18);
                clawServoR.setPosition(0.8);
            }
            if (moveForward){
                //https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html?com/qualcomm/robotcore/hardware/DcMotorEx.html
                // setMode(RunMode), DcMotor.RunMode.RUN_TO_POSITION, getTargetPosition(), isBusy()
                //leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                long distance = 500;
                long initialL = leftDrive.getCurrentPosition();
                long initialR = rightDrive.getCurrentPosition();
                long destL = initialL + distance;
                long destR = initialR + distance;

                leftDrive.setPower(1.0);
                rightDrive.setPower(1.0);

                leftDrive.setTargetPosition((int) destL);
                rightDrive.setTargetPosition((int) destR);

                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (leftDrive.isBusy() || rightDrive.isBusy() ){
                    sleep(100);
                }

                //leftDrive.setMode(originalLeftMode);
                //rightDrive.setMode(originalRightMode);
                leftDrive.setPower(0.0);
                rightDrive.setPower(0.0);
                leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // Show the elapsed game time and wheel power.
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
            telemetry.addData("Motors", "leftPower (%.2f), rightPower (%.2f)", leftPower, rightPower);
            telemetry.addData("Motors Pos", "leftPos (%d), rightPos (%d)", e1, e2);
            telemetry.addData("Motors Velocity", "leftV (%f), rightV (%f)", v1, v2);
            telemetry.addData("Elevator", "elevatorPow (%.2f)", elevator_power);
            telemetry.addData("Elevator Pos", "Pos (%d)", e3);
            telemetry.addData("Elevator Velocity", "V (%f)", v3);
            telemetry.addData("ClawValue Dh", "LeftPos (%f) RightPos (%f)", claw_value_left, claw_value_right);
            telemetry.update();
        }
    }
}
