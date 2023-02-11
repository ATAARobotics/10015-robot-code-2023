package org.firstinspires.ftc.teamcode;

import java.util.List;
import java.util.ArrayList;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor;

import com.qualcomm.hardware.bosch.BNO055IMU;

// this is just a dumb wrapper
//import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.controller.PIDController;

// our other classes, for 10015
import org.firstinspires.ftc.teamcode.DriveBase;
import org.firstinspires.ftc.teamcode.Elevator;
import org.opencv.core.Mat;
import org.opencv.objdetect.QRCodeDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

//import org.firstinspires.ftc.teamcode.OpenCv.FindQrCodePipeline;



class FindQrCodePipeline extends OpenCvPipeline
{
    public LinearKiwiAutonomous parent = null;
    public String found_name = null;
    private QRCodeDetector qr = new QRCodeDetector();

    @Override
    public Mat processFrame(Mat input)
    {
        found_name = qr.detectAndDecodeCurved(input);
        parent.telemetry.addData("detected" , found_name);
        if (found_name != null && !found_name.isEmpty()) {
            parent.the_code = found_name;
        }
        return input;
    }
}


@Autonomous(name="Kiwi: Linear Autonomous (RIGHT)", group="Autonomous")
public class LinearKiwiAutonomous extends LinearOpMode {

    private DriveBase drivebase = null;
    private Elevator elevator = null;

    private ColorSensor colour = null;
    private DistanceSensor distance = null;
    public String the_code = null;

    private OpenCvCamera camera = null;


    private void ensure_stop(double heading) {
        /// ideally shouldn't need this, but .. here we are
        drivebase.drive.driveFieldCentric(0.0, 0.0, 0.0, heading);
        drivebase.motor_left.set(0.0);
        drivebase.motor_right.set(0.0);
        drivebase.motor_slide.set(0.0);
    }

    public class Action {
        public double start_time = -1.0;

        public void start(double t, DriveBase drivebase, Elevator elevator) {
            start_time = t;
        }

        public void do_drive(double heading, DriveBase drivebase) {
            drivebase.drive.driveFieldCentric(0.0, 0.0, 0.0, heading);
        }

        public void do_elevator(Elevator elevator) {
            if (!elevator.motor_elevator.atTargetPosition()) {
                elevator.motor_elevator.set(0.15);
            }
        }

        public boolean is_done(double time) {
            return false;
        }
    }

    public class ElevatorAction extends Action {
        public int target = 0;

        public ElevatorAction(int t) {
            target = t;
        }

        public void start(double t, DriveBase drivebase, Elevator elevator) {
            start_time = t;
            elevator.motor_elevator.setRunMode(Motor.RunMode.PositionControl);
            elevator.motor_elevator.setTargetPosition(target);
        }

        public void do_drive(double heading, DriveBase drivebase) {
            drivebase.drive.driveFieldCentric(0.0, 0.0, 0.0, heading);
        }

        public void do_elevator(Elevator elevator) {
            if (!elevator.motor_elevator.atTargetPosition()) {
                elevator.motor_elevator.set(0.15);
            }
        }

        public boolean is_done(double time) {
            if (elevator.motor_elevator.atTargetPosition() || time - start_time > 1.5) {
                return true;
            }
            return false;
        }
    }

    public class ClawAction extends Action {
        public boolean closed = false;

        public ClawAction() {
        }

        public void do_drive(double heading, DriveBase drivebase) {
            drivebase.drive.driveFieldCentric(0.0, 0.0, 0.0, heading);
        }

        public void do_elevator(Elevator elevator) {
            if (!closed) {
                closed = true;
                elevator.open_claw();
            }
            if (!elevator.motor_elevator.atTargetPosition()) {
                elevator.motor_elevator.set(0.15);
            }
        }

        public boolean is_done(double time) {
           if (time - start_time > 0.5) {
                return true;
            }
            return false;
        }
    }

    public class WaitAction extends Action {
        public double duration = 0.0;

        public WaitAction(double d) {
            duration = d;
        }

        public void do_drive(double heading, DriveBase drivebase) {
            drivebase.drive.driveFieldCentric(0.0, 0.0, 0.0, heading);
        }

        public void do_elevator(Elevator elevator) {
            if (!elevator.motor_elevator.atTargetPosition()) {
                elevator.motor_elevator.set(0.15);
            }
        }

        public boolean is_done(double time) {
            if (time >= start_time + duration) {
                return true;
            }
            return false;
        }
    }

    public class DriveAction extends Action {
        public double stick_x = 0.0;
        public double stick_y = 0.0;
        public double turn = 0.0;
        public double duration = 0.0;

        public DriveAction(double x, double y, double t, double dur) {
            stick_x = x;
            stick_y = y;
            turn = t;
            duration = dur;
        }

        public void do_drive(double heading, DriveBase drivebase) {
            drivebase.drive.driveFieldCentric(stick_x, stick_y, turn, heading);
        }

        public boolean is_done(double time) {
            if ((time - start_time) > duration) {
                return true;
            }
            return false;
        }
    }

    public class TurnAction extends Action {
        public double stick_x = 0.0;
        public double stick_y = 0.0;
        public double turn = 0.0;
        public double target_heading = 0.0;
        public boolean done = false;

        public TurnAction(double t, double target) {
            stick_x = 0.0;
            stick_y = 0.0;
            turn = t;
            target_heading = target;
        }

        public void do_drive(double heading, DriveBase drivebase) {
            drivebase.drive.driveFieldCentric(stick_x, stick_y, turn, heading);
            if (heading < target_heading) {
                done = true;
            }
        }

        public boolean is_done(double time) {
            return done;
        }
    }

    public class OtherTurnAction extends Action {
        public double stick_x = 0.0;
        public double stick_y = 0.0;
        public double turn = 0.0;
        public double target_heading = 0.0;
        public boolean done = false;

        public OtherTurnAction(double t, double target) {
            stick_x = 0.0;
            stick_y = 0.0;
            turn = t;
            target_heading = target;
        }

        public void do_drive(double heading, DriveBase drivebase) {
            drivebase.drive.driveFieldCentric(stick_x, stick_y, turn, heading);
            if (heading > target_heading) {
                done = true;
            }
        }

        public boolean is_done(double time) {
            return done;
        }
    }

    public class DetermineCodeAction extends Action {
        public double duration = 0.0;
        public LinearKiwiAutonomous auto = null;
        public String code = "unknown";
        public boolean started = false;
        FindQrCodePipeline pipeline = new FindQrCodePipeline();


        public DetermineCodeAction(LinearKiwiAutonomous a, double d) {
            duration = d;
            auto = a;
            pipeline.parent = a;
        }

        public void do_drive(double heading, DriveBase drivebase) {
            drivebase.drive.driveFieldCentric(0.0, 0.0, 0.0, heading);
            if (!started) {
                started = true;
                camera.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
                camera.setPipeline(pipeline);
            }
        }

        public boolean is_done(double time) {
            if ((time - start_time) > duration) {
                return true;
            }
            if (auto.the_code != null) {
                return true;
            }
            return false;
        }
    }

    public void add_todo_list_post_detection(List<Action> todo, double field_factor, int code_number) {
        todo.add(new DriveAction(0.0, -0.5, 0.0, 1.5 * field_factor)); // north
        todo.add(new WaitAction(1.0));
        todo.add(new DriveAction(-0.5, 0.0, 0.0, 2.0 * field_factor));
        todo.add(new DriveAction(0.0, -0.5, 0.0, 0.85 * field_factor)); // north
        todo.add(new ElevatorAction(1650)); //go to high position
        todo.add(new DriveAction(-0.5, 0.0, 0.0, 0.9 * field_factor));//left
        todo.add(new ClawAction()); //open
        todo.add(new DriveAction(0.5,0.0,0,0.8 * field_factor));
        todo.add(new ElevatorAction(300)); //go to drive position
        todo.add(new DriveAction(0.0, -0.5, 0.0, 1.0 * field_factor)); // north
        //todo.add(new OtherTurnAction(0.2, 189.0));  // turn around (so we drive forwards)

        if (code_number == 1) {
            // already there
        } else if (code_number == 2) {
            todo.add(new DriveAction(0.5, 0, 0.0, 2.0 * field_factor));
        } else {
            //todo.add(new DriveAction(0.8, 0.0, 0.0, 3.8 * field_factor));
            // turbo?
            todo.add(new DriveAction(0.7, 0.0, 0.0, 2.5 * field_factor));
        }
    }

    @Override
    public void runOpMode() {
        //
        // setup
        //

        telemetry.addData("status", "startup");
        telemetry.update();

        colour = hardwareMap.get(ColorSensor.class, "colour");
        colour.enableLed(true);

        distance = hardwareMap.get(DistanceSensor.class, "colour");

        // let the drivebase set itself up
        drivebase = new DriveBase(hardwareMap);

        // elevator setup
        elevator = new Elevator(hardwareMap);

        telemetry.addData("status", "initialized");
        telemetry.update();

        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
        camera.openCameraDevice();
        //
        // done setup wait for start
        //

        waitForStart();

        //
        // start has been pressed
        //

        // make sure robot starts at correct position
        drivebase.imu.resetYaw();
        // ensure we have "zero" at the bottom of our elevator
        elevator.motor_elevator.resetEncoder();
        elevator.motor_elevator.setRunMode(Motor.RunMode.PositionControl);
        elevator.open_claw();
        sleep(1000);
        elevator.close_claw();
        sleep(1000);

        //
        // main logic loop
        //

        List<Action> todo = new ArrayList<Action>();

        double heading = 0.0;

        // account for extra friction in field.
        double field_factor = 1.2;

        // drive ahead, slowly, for a little while
        todo.add(new ElevatorAction(300));
        todo.add(new DriveAction(0.0, -0.5, 0.0, .5 * field_factor)); // ahead
        todo.add(new TurnAction(-0.2, -89.0));  // turn to line up sensor
        todo.add(new DriveAction(-0.4, 0.0, 0.0, 0.8 * field_factor)); // strafe a bit
        todo.add(new DetermineCodeAction(this, 5.0));

        drivebase.reset();
        while (!todo.isEmpty() && opModeIsActive()) {
            telemetry.addData("todo", todo.size());
            Action doing = todo.get(0);
            todo.remove(doing);
            doing.start(time, drivebase, elevator);
            while (!doing.is_done(time) && opModeIsActive()) {
                heading = - drivebase.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                heading += 0.9; // we think there is absolute error
                doing.do_drive(heading, drivebase);
                doing.do_elevator(elevator);
                telemetry.addData("start_time", doing.start_time);
                telemetry.addData("time", time);
                telemetry.addData("duration", time - doing.start_time);
                telemetry.addData("heading", heading);
                telemetry.addData(
                    "encoders",
                    String.format(
                        "left=%d right=%d slide=%d",
                        drivebase.motor_left.encoder.getPosition(),
                        drivebase.motor_right.encoder.getPosition(),
                        drivebase.motor_slide.encoder.getPosition()
                    )
                );
                telemetry.addData("detected", the_code);
                telemetry.update();
            }
        }

        //
        // translate code
        //

        int code_number = -1;
        try {
            code_number = Integer.parseInt(the_code);
        } catch (NumberFormatException e) {
        }
        telemetry.addData("code", code_number);

        add_todo_list_post_detection(todo, field_factor, code_number);

        while (!todo.isEmpty() && opModeIsActive()) {
            telemetry.addData("todo", todo.size());
            telemetry.addData("code", code_number);
            Action doing = todo.get(0);
            todo.remove(doing);
            doing.start(time, drivebase, elevator);
            while (!doing.is_done(time) && opModeIsActive()) {
                heading = - drivebase.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                heading += 0.9;
                doing.do_drive(heading, drivebase);
                doing.do_elevator(elevator);
                telemetry.update();
            }
        }

        // done
        ensure_stop(heading);
        elevator.motor_elevator.setRunMode(Motor.RunMode.PositionControl);
        elevator.motor_elevator.setTargetPosition(10);
        double timeout = time + 2.0;
        while (!elevator.motor_elevator.atTargetPosition() && opModeIsActive() && time < timeout) {
            // XXX why does this ever successfuly go "down" at all??
            elevator.motor_elevator.set(0.15);
            heading = - drivebase.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            heading += 0.9;
            drivebase.drive.driveFieldCentric(0.0, 0.0, 0.0, heading);
        }
        elevator.motor_elevator.setRunMode(Motor.RunMode.RawPower);
    }
}
