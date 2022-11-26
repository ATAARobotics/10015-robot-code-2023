package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class DemoRobotInterface {
    public DcMotor frm = null;
    public DcMotor flm = null;
    public DcMotor elevatorm = null;
    public Servo clawF = null;
    public Servo clawS = null;

    public DemoRobotInterface(HardwareMap hardwareMap, Telemetry telemetry) {
        frm = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        flm = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        elevatorm = hardwareMap.get(DcMotor.class, "Elevator");
        clawF = hardwareMap.get(Servo.class, "ClawServoOne");
        clawS = hardwareMap.get(Servo.class, "ClawServoTwo");

    }
}
