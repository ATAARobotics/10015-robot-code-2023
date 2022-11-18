/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 *
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class BasicLinearOpMode extends LinearOpMode {
    // Declare OpMode motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;
    private DcMotorEx elevatorDrive = null;
    //private NormalizedColorSensor colorSensor = null;

    // declare servos
    private Servo clawServoL = null;
    private Servo clawServoR = null;

    // used for query information
    private long      e1, e2, e3, e4;
    private double    v1, v2, v3, v4, claw_value_left, claw_value_right;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
        //colorSensor = hardwareMap.get(NormalizedColorSensor.class, "imu");

        //int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        //final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        //float colour_gain = 3;
        //colorSensor.setGain(colour_gain);
        // Once per loop, we will update this hsvValues array. The first element (0) will contain the
        // hue, the second element (1) will contain the saturation, and the third element (2) will
        // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
        // for an explanation of HSV color.
        //final float[] hsvValues = new float[3];

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        elevatorDrive.setDirection(DcMotor.Direction.FORWARD);
        //elevatorDrive.setTargetPosition(1);

        clawServoL.setDirection(Servo.Direction.FORWARD);
        clawServoR.setDirection(Servo.Direction.FORWARD);
        clawServoL.scaleRange(0.0,1.0);
        clawServoR.scaleRange( 0.0, 1.0);

        clawServoL.setPosition(1.0);
        clawServoR.setPosition(0.0);

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
            double turn  =  gamepad1.right_stick_x;
            double elevator_up = gamepad2.left_trigger;
            double elevator_down = -gamepad2.right_trigger;
            boolean claw = gamepad2.cross;
            boolean claw_release = gamepad2.square;
            boolean moveForward = gamepad1.circle;
            boolean rotate = gamepad1.triangle;

            boolean dPadLeft = gamepad1.dpad_left;
            boolean dPadRight = gamepad1.dpad_right;
            boolean dPadUp = gamepad1.dpad_up;
            boolean dPadDown = gamepad1.dpad_down;


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

            if (elevator_power / 0.5 >0.0){
                elevatorDrive.setDirection(DcMotor.Direction.REVERSE);
                elevator_power = elevator_power/2;
            }
            elevatorDrive.setPower(elevator_power / 0.5);

            // claw on
            if (claw) {
                clawServoL.setPosition(0.25);
                clawServoR.setPosition(0.75);
            }
            // claw off
            if (claw_release){
                clawServoL.setPosition(1.0);
                clawServoR.setPosition(0.0);
            }
            if (dPadUp){
                // set elevator to 3 feet (-752)
                elevatorDrive.setPower(0.5);
                elevatorDrive.setTargetPosition((int) -752);
                elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (elevatorDrive.isBusy()){
                    sleep(100);
                }

                elevatorDrive.setPower(0.0);
                elevatorDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevatorDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            if (dPadDown){
                // set elevator to zero (10)
                elevatorDrive.setPower(0.0);

                elevatorDrive.setTargetPosition((int) 0);
                elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevatorDrive.setPower(0.5);
                while (elevatorDrive.isBusy()){
                    sleep(100);
                }
                // stop and reset to zero
                //elevatorDrive.setPower(0.0);
                elevatorDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevatorDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevatorDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            if (dPadLeft){
                // set elevator to one foot (-253)
                elevatorDrive.setPower(0.5);
                elevatorDrive.setTargetPosition((int) -253);
                elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (elevatorDrive.isBusy()){
                    sleep(100);
                }

                elevatorDrive.setPower(0.0);
                elevatorDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevatorDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            if (dPadRight){
                // set elevator to 2 feet (-517)
                elevatorDrive.setPower(0.5);
                elevatorDrive.setTargetPosition((int) -517);
                elevatorDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (elevatorDrive.isBusy()){
                    sleep(100);
                }

                elevatorDrive.setPower(0.0);
                elevatorDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevatorDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            if (moveForward){
                //https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html?com/qualcomm/robotcore/hardware/DcMotorEx.html
                // setMode(RunMode), DcMotor.RunMode.RUN_TO_POSITION, getTargetPosition(), isBusy()
                //leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                long distance = 528;
                long initialL = leftDrive.getCurrentPosition();
                long initialR = rightDrive.getCurrentPosition();
                long destL = initialL + distance;
                long destR = initialR + distance;

                leftDrive.setTargetPosition((int) destL);
                rightDrive.setTargetPosition((int) destR);

                leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftDrive.setPower(0.60);
                rightDrive.setPower(0.60);

                while (leftDrive.isBusy() || rightDrive.isBusy() ){
                    sleep(100);
                }

                leftDrive.setPower(0.0);
                rightDrive.setPower(0.0);
                leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (rotate){
                // add comment
                long initialL = leftDrive.getCurrentPosition()+266;
                long initialR = rightDrive.getCurrentPosition()-266;

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
                leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

            // get sensor values + normalise using Color class
            //NormalizedRGBA colors = colorSensor.getNormalizedColors();
            //Color.colorToHSV(colors.toColor(), hsvValues);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.

            //relativeLayout.post(new Runnable() {
            //    public void run() {
            //        relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, hsvValues));
            //    }
            //});
            /*
            String colorVal = "None";
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
            */



            telemetry.addData("BasicLinearOpMode Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftPower (%.2f), rightPower (%.2f)", leftPower, rightPower);
            telemetry.addData("Motors Pos", "leftPos (%d), rightPos (%d)", e1, e2);
            telemetry.addData("Motors Velocity", "leftV (%f), rightV (%f)", v1, v2);
            telemetry.addData("Elevator", "elevatorPow (%.2f)", elevator_power);
            telemetry.addData("Elevator Pos", "Pos (%d)", e3);
            telemetry.addData("Elevator Velocity", "V (%f)", v3);
            //telemetry.addData("Color", "name (%d)" + colorVal);
            //telemetry.addData("Color Values ", "r (%.3f) g (%.3f) b (%.3f)", colors.red, colors.green, colors.blue);
            telemetry.addData("ClawValue D", "LeftPos (%f) RightPos (%f)", claw_value_left, claw_value_right);
            telemetry.update();
        }
    }
}
