package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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



    @Override
    public void runOpMode() throws InterruptedException {
        robotui = new DemoRobotInterface(hardwareMap, telemetry);

        waitForStart();

        robotui.clawS.setDirection(Servo.Direction.REVERSE);

        robotui.frm.setDirection(DcMotorSimple.Direction.REVERSE);
        robotui.flm.setDirection(DcMotorSimple.Direction.REVERSE);




        while (opModeIsActive())
        {
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
