package org.firstinspires.ftc.teamcode;

import static java.lang.System.currentTimeMillis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Date;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoPeriod_1", group="Autonomous")

public class Autonomous_1 extends LinearOpMode {
    private DemoRobotInterface robotui = null;

    public float MotorsSpeed = 0.1f;

    public float wait = 3.0f;



    @Override
    public void runOpMode() throws InterruptedException {
        robotui = new DemoRobotInterface(hardwareMap, telemetry);

        long set;
        long setO;

        waitForStart();

        robotui.elevatorm.setTargetPosition(1300);
        robotui.elevatorm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotui.elevatorm.setPower(-0.5f);

        robotui.frm.setPower(0.2);
        robotui.flm.setPower(-0.2);

        set = currentTimeMillis();
        //setO = currentTimeMillis()*2;

        while (opModeIsActive())
        {
            if (set + 1350 < currentTimeMillis()) {
                robotui.frm.setPower(0);
                robotui.flm.setPower(0);
            }

            /*if (setO + 3000 < currentTimeMillis()) {
                telemetry.addData("State", "STOP");
                telemetry.update();
                robotui.frm.setPower(0f);
                robotui.flm.setPower(0f);
                robotui.elevatorm.setTargetPosition(0);
            }*/
        }
    }
}
