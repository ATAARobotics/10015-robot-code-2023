package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="August20", group="TeleOp")

public class August20 extends LinearOpMode {
    private DemoRobotInterface robotui = null;
    public float axisX;
    public float axisY;

    public float acceleration = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        robotui = new DemoRobotInterface(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive())
        {
            axisX = clamp(-gamepad1.left_stick_y, -0.5f, 0.5f);
            axisY = clamp(-gamepad1.left_stick_x, -0.5f, 0.5f);

            /*if(axisY != 0 && axisX != 0) DOES NOT EVEN A THING
            {
                if(axisX>0.1)
                {
                    robotui.flm.setPower(axisY);
                    robotui.frm.setPower(axisY*axisX);
                }

                else if(axisX<0.1)
                {
                    robotui.flm.setPower(axisY*axisX*-1);
                    robotui.frm.setPower(axisY);
                }
            }

            else*/

            if((axisX <0.1)&&(axisX>-0.1)) // turn
            {
                robotui.flm.setPower(axisY); // axisY/Math.abs(axisY)
                robotui.frm.setPower(axisY);
            }

            else if((axisY <0.1)&&(axisY>-0.1)) // go fwrds/bckwrds
            {
                robotui.flm.setPower(axisX);
                robotui.frm.setPower(-axisX);
            }

            else if(false)
            {

            }
        }
    }

    public static float clamp(final float value, final float min, final float max) {
        return (value < min) ? min : ((value > max) ? max : value);
    }
}
