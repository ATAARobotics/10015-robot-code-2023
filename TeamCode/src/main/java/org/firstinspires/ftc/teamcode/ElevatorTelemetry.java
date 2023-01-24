package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

    @TeleOp(name = "ElevatorTelemetry ", group = "TeleOp")

    public class ElevatorTelemetry extends LinearOpMode {
        private DemoRobotInterface robotui = null;

        public int lowPos = 200;
        public int medPos = 1000;
        public int highPos = 1800;

        public int limitUP = 9000;
        public int limitDN = 0;

        public float speed = 0.25f;


        @Override
        public void runOpMode() throws InterruptedException {
            robotui = new DemoRobotInterface(hardwareMap, telemetry);
            robotui.elevatorm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robotui.elevatorm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            waitForStart();

            while (opModeIsActive())
            {
                if(gamepad2.dpad_up)
                    {
                        robotui.elevatorm.setPower(speed);
                    }
                else if(gamepad2.dpad_down)
                    {
                        robotui.elevatorm.setPower(-speed);
                    }
                else
                    {
                        robotui.elevatorm.setPower(0);
                    }
                telemetry.addData("Encoder", robotui.elevatorm.getCurrentPosition());
                telemetry.update();
                }
            }
        }