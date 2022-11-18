/*
CURRENT SETUP:
currently we are using Motorola moto E (4) phones released 2017
https://www.gsmarena.com/motorola_moto_e4-8721.php
These have Android 7.1.1 installed.
The controllers are using firmware 1.8.2 (current latest)

test a control hub on a Windows machine using browser:
https://www.youtube.com/watch?v=fyxpptqQumw

OMNIDRIVE SPECIFIC:
http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf
https://www.chiefdelphi.com/t/pic-kiwi-vex-robot/71688/9
https://www.reddit.com/r/FTC/comments/ilsj6z/trying_to_code_a_3_omniwheel_drive_robot_using/

Rotating a wheel a certain distance "w" means w = (wheel radius) * (angle wheel rotates around
its axle in radians).
For 1 cm of forward motion, the individual wheel distances needed are : front = 0;
right = +sqrt(3)/2 cm; left = -sqrt(3)/2 cm.
For 1 cm of rightward motion, the individcual wheel distances needed are: front = -1 cm;
right = +0.5 cm; left = +0.5 cm.

Currently all that available from REV for Omni wheels is 60mm wheels
https://www.revrobotics.com/DUO-Omni-Wheels/
60mm == 2.3622 inches
90mm == 3.54331 inches (these are the ones we have 2 of)

basic setup:
https://www.youtube.com/watch?v=CdcpNZzekb0
GamepadInfo:
https://ftcforum.firstinspires.org/forum/first-tech-challenge-community-forum-this-is-an-open-forum/teams-helping-teams-programming/77647-ftc-game-controller-programming-reference


 */

package org.firstinspires.ftc.teamcode;

import android.app.ActionBar;
import android.app.Activity;
import android.app.ActivityManager;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.content.res.Configuration;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.os.IBinder;
import android.preference.PreferenceManager;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.StringRes;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.webkit.WebView;
import android.widget.ImageButton;
import android.widget.LinearLayout;
import android.widget.LinearLayout.LayoutParams;
import android.widget.PopupMenu;
import android.widget.TextView;

import com.google.blocks.ftcrobotcontroller.ProgrammingWebHandlers;
import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;
import com.qualcomm.ftccommon.ClassManagerFactory;
import com.qualcomm.ftccommon.FtcAboutActivity;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.ftccommon.FtcEventLoopIdle;
import com.qualcomm.ftccommon.FtcRobotControllerService;
import com.qualcomm.ftccommon.FtcRobotControllerService.FtcRobotControllerBinder;
import com.qualcomm.ftccommon.FtcRobotControllerSettingsActivity;
import com.qualcomm.ftccommon.LaunchActivityConstantsList;
import com.qualcomm.ftccommon.LaunchActivityConstantsList.RequestCode;
import com.qualcomm.ftccommon.Restarter;
import com.qualcomm.ftccommon.UpdateUI;
import com.qualcomm.ftccommon.configuration.EditParameters;
import com.qualcomm.ftccommon.configuration.FtcLoadFileActivity;
import com.qualcomm.ftccommon.configuration.RobotConfigFile;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
// BuildConfig is currently unusable - perhaps use : com.qualcomm.ftccommon.configuration
//import com.qualcomm.ftcrobotcontroller.BuildConfig;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.HardwareFactory;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.eventloop.opmode.FtcRobotControllerServiceState;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.Utility;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.util.ClockWarningSource;
import com.qualcomm.robotcore.util.Device;
import com.qualcomm.robotcore.util.Dimmer;
import com.qualcomm.robotcore.util.ImmersiveMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.WebServer;
import com.qualcomm.robotcore.wifi.NetworkConnection;
import com.qualcomm.robotcore.wifi.NetworkConnectionFactory;
import com.qualcomm.robotcore.wifi.NetworkType;

import org.firstinspires.ftc.ftccommon.external.SoundPlayingRobotMonitor;
import org.firstinspires.ftc.ftccommon.internal.AnnotatedHooksClassFilter;
import org.firstinspires.ftc.ftccommon.internal.FtcRobotControllerWatchdogService;
import org.firstinspires.ftc.ftccommon.internal.ProgramAndManageActivity;
import org.firstinspires.ftc.onbotjava.ExternalLibraries;
import org.firstinspires.ftc.onbotjava.OnBotJavaHelperImpl;
import org.firstinspires.ftc.onbotjava.OnBotJavaProgrammingMode;
import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;
import org.firstinspires.ftc.robotcore.internal.hardware.android.AndroidBoard;
import org.firstinspires.ftc.robotcore.internal.network.DeviceNameManager;
import org.firstinspires.ftc.robotcore.internal.network.DeviceNameManagerFactory;
import org.firstinspires.ftc.robotcore.internal.network.PreferenceRemoterRC;
import org.firstinspires.ftc.robotcore.internal.network.StartResult;
import org.firstinspires.ftc.robotcore.internal.network.WifiDirectChannelChanger;
import org.firstinspires.ftc.robotcore.internal.network.WifiMuteEvent;
import org.firstinspires.ftc.robotcore.internal.network.WifiMuteStateMachine;
import org.firstinspires.ftc.robotcore.internal.opmode.ClassManager;
import org.firstinspires.ftc.robotcore.internal.opmode.OnBotJavaHelper;
import org.firstinspires.ftc.robotcore.internal.system.AppAliveNotifier;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.robotcore.internal.system.PreferencesHelper;
import org.firstinspires.ftc.robotcore.internal.system.ServiceController;
import org.firstinspires.ftc.robotcore.internal.ui.ThemedActivity;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;
import org.firstinspires.ftc.robotcore.internal.webserver.RobotControllerWebInfo;
import org.firstinspires.ftc.robotserver.internal.programmingmode.ProgrammingModeManager;
import org.firstinspires.inspection.RcInspectionActivity;
import org.threeten.bp.YearMonth;
import org.xmlpull.v1.XmlPullParserException;

import java.io.FileNotFoundException;
import java.util.Arrays;
import java.util.Date;
import java.lang.Math;

import java.util.List;
import java.util.Queue;
import java.util.Vector;
import java.util.concurrent.ConcurrentLinkedQueue;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vec4F;


// https://www.chiefdelphi.com/t/pic-kiwi-vex-robot/71688/8
// copied from here. This is a C attempt at the kiwi drive

@TeleOp(name="Basic: KiwiDrive OpMode", group="Opmode")
class KiwiDrive extends OpMode {

    public Gamepad gamepad1;

    public DcMotor north;
    public DcMotor southEast;
    public DcMotor southWest;

    public Date startTime = new Date();
    public int min_value = 0;
    public int max_value = 1;
    public int pad_signal_offset = 254;

    public float rot = 254 - gamepad1.right_stick_x;
    public boolean run;
    public double robotCenter_to_wheelCenter = 1.0;

    //Limit vector length to 127
    //trans_x = (37+(254*trans_x2)/359);
    //trans_y = (37+(254*trans_y2)/359);

    // limits an input value to upper and lower bounds.
    public double limit( double value, double min, double max) {
        if (value < min) {
            value = min;
        } else if (value > max) {
            value = max;
        }
        return value;
    }

    @Override
    public void init() {
        // set up motors and gamepad. Set gamepad light to green
        // to let user know init is good. go to red if error
        try {
            this.run = true;
            north = hardwareMap.get(DcMotor.class, "north");
            southEast = hardwareMap.get(DcMotor.class, "southEast");
            southWest = hardwareMap.get(DcMotor.class, "southWest");

            gamepad1 = hardwareMap.get(Gamepad.class, "gamepad1");
            gamepad1.setLedColor(0.0,1.0,0.0, 100);
        } catch (Exception e){
            gamepad1.setLedColor(1.0,0.0,0.0, 100);
        }
    }


    /*
    Make sure variables are unsigned ints... rollovers WILL occur with signed ints
    Grab inputs (x,y coords in rectangular coordinates and rotation rate)
    I am guessing it can take a PWM signal of +/-255,
    PWM signals are between -1 and +1 with 0 as no operation
    public float trans_x2 = 254 - PWM_in1;
    public float trans_y2 = 254 - PWM_in2;
     */
    public void translate(Vec3F joypad_vector) {
        try {
            //Assign and limit wheel outputs
            // unpack data
            Vec3f vector_item = joypad_vector.getData();
            double northSouth = joypad_vector.getData()[0];
            double eastWest = joypad_vector.getData()[1];
            double rotation = joypad_vector.getData()[2];

            // work out values limiting values
            double v1 = (double) (((2 * n) + rot) / 3);
            double v2 = (double) (601 + rot - n - (168 * se / 97) / 3);
            double v3 = (double) ((161 + rot - n + (168 * se / 97)) / 3);


            double wheel1 = this.limit((double) v1, 0.0, 254.0);
            double wheel2 = this.limit(v2, 0.0, 254.0);
            double wheel3 = this.limit(v3, 0.0, 254.0);
            //wheel_1  = (unsigned char)limit( (((2*trans_x) + rot) / 3), 0, 254);
            //wheel_2 = (unsigned char)limit( ((601 + rot - trans_x - (168*trans_y/97)) / 3), 0, 254);
            //wheel_3 = (unsigned char)limit( ((161 + rot - trans_x + (168*trans_y/97)) / 3) , 0, 254);

            // turn on motors using values
            north.setPower(wheel1);
            southEast.setPower(wheel2);
            southWest.setPower(wheel3);

        } catch(Exception e) {
            System.out.println("translate function - Something went wrong.");
            String traceback = Arrays.toString(Thread.currentThread().getStackTrace());
            System.out.println(traceback);
            }

    }

    public void translate_1CM_North() {
        double wheel1 = 0.0;
        double wheel2 = java.lang.Math.sqrt(3)/2;
        double wheel3 = wheel2*-1;
        north.setPower(wheel1);
        southEast.setPower(wheel2);
        southWest.setPower(wheel3);
    }
    public void translate_1CM_South() {
        double wheel1 = 0.0;
        double wheel3 = java.lang.Math.sqrt(3)/2;
        double wheel2 = wheel3*-1;
        north.setPower(wheel1);
        southEast.setPower(wheel2);
        southWest.setPower(wheel3);
    }
    public void translate_1CM_Right() {
        double wheel1 = -1.0;
        double wheel2 = 0.5;
        double wheel3 = 0.5;
        north.setPower(wheel1);
        southEast.setPower(wheel2);
        southWest.setPower(wheel3);
    }
    public void translate_1CM_Left() {
        double wheel1 = 1.0;
        double wheel2 = -0.5;
        double wheel3 = -0.5;
        north.setPower(wheel1);
        southEast.setPower(wheel2);
        southWest.setPower(wheel3);
    }
    public void rotate_CW() {
        double wheel1 = this.robotCenter_to_wheelCenter*10*-1;
        double wheel2 = this.robotCenter_to_wheelCenter*10*-1;
        double wheel3 = this.robotCenter_to_wheelCenter*10*-1;
        north.setPower(wheel1);
        southEast.setPower(wheel2);
        southWest.setPower(wheel3);
    }
    public void rotate_CCW() {
        double wheel1 = this.robotCenter_to_wheelCenter*10;
        double wheel2 = this.robotCenter_to_wheelCenter*10;
        double wheel3 = this.robotCenter_to_wheelCenter*10;
        north.setPower(wheel1);
        southEast.setPower(wheel2);
        southWest.setPower(wheel3);
    }

    @Override
    public void loop() {
        //while
        //waitForStart();
        //opModeIsActive()
        //double time = OpMode.time;

        while (this.run) {
            this.north.setPower(0.5);
            this.southEast.setPower(0.5);
            this.southWest.setPower(0.5);

            Vec3F pad_direction = new Vec3F(0.0f, 0.0f, 0.0f);
            // Gamepad.GamepadCallback is pretty much useless
            //public void translate(Vec3F joypad_vector) {

            pad_direction = query_gamepad_vector(this.translate());

            boolean stop_btn_press = gamepad1.x;
            if (stop_btn_press){
                this.run = false;
                requestOpModeStop();
            }

            //update telemetry
            internalPostLoop();
        }
    }


    public Vec3F query_gamepad_vector(translate_callback){
        //Analog sticks are represented as floats that range from -1.0 to +1.0.
        // They will be 0.0 while at rest.
        // The horizontal axis is labeled x, and the vertical axis is labeled y.
        Vec3F result = new Vec3F(0.0f, 0.0f, 0.0f);

        float trans_x = self.pad_signal_offset - gamepad1.left_stick_x;
        float trans_y = self.pad_signal_offset - gamepad1.left_stick_y;
        float rotation_x = pad_signal_offset - gamepad1.right_stick_x;
        //float rotation_y = pad_signal_offset - gamepad1.right_stick_y;

        //Limit vector length to 127
        trans_x = (37 + (self.pad_signal_offset * trans_x)/359);
        trans_y = (37 + (self.pad_signal_offset * trans_y)/359);
        rotation_x = (37 + (self.pad_signal_offset * rotation_x)/359);
        float[] items = { trans_x, trans_y, rotation_x};
        result.setData( items );
        translate_callback(result);
        return result;
    }

}
