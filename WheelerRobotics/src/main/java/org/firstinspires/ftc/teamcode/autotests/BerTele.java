package org.firstinspires.ftc.teamcode.autotests;

import static org.firstinspires.ftc.teamcode.robot.boats.Bert.armPlacePos;
import static org.firstinspires.ftc.teamcode.robot.boats.Bert.slidePickupPos;
import static org.firstinspires.ftc.teamcode.robot.boats.Bert.slidePlacePos;
import static org.firstinspires.ftc.teamcode.robot.boats.Bert.tiltPlacePos;
import static java.lang.Math.abs;

import android.content.Context;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.PropAprilDet;
import org.firstinspires.ftc.teamcode.robot.boats.Bert;

@Config
@TeleOp
public class BerTele extends LinearOpMode {
    public static double aPos = 0.3;
    public static double tPos = 0;
    public static boolean cPos = false;
    public static boolean pPos = false;

    public static double target = 0;


    /*

    Controller 1:
    Left stick: XY drive
    Right stick: X turn
    Triggers: Intake, but only if player 2 is not controlling it
    Right Bumper: (optional) april tag backdrop approach deceleration


    Controller 2:
    Right Stick Y: Slides
    Left Stick Y: Intake (down -> in, up -> out)
        - (optional) if intake maxed, then activate B macro
    A: macro
        - Slides to place (pretty high)
        - Arm/Tilt to place
        - wait, then Claw open
    B: macro
        - Everything to pickup pos
    X: toggle claw
    Y: toggle arm/tilt position
    Dpad up + right bumper: launch plane, wait reset servo
    Right Bumper + Left Bumper: Joystick Hangs
        - Right stick Y: right hang
        - Left stick Y: left hang
    Right Trigger: Both Hangs Up
    Left Trigger: Both Hangs Down

    Macro Canceling Events:
        - Joysticks used to move slides/intake
        - another macro activated
        - claw/arm/tilt toggled




     */
    public boolean xWasPressed = false;
    public boolean yWasPressed = false;
    public boolean aWasPressed = false;
    public boolean bWasPressed = false;
    public ElapsedTime aMacro = new ElapsedTime();
    public ElapsedTime planeCooldown = new ElapsedTime();
    public boolean planeLaunched = false;
    public boolean aMacroing = false;
    public boolean bMacroing = false;
    public Bert b;
    public int kcount = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        b = new Bert();
        b.init(hardwareMap);
        PropAprilDet ad = new PropAprilDet();
        ad.init(hardwareMap, "Webcam 2");

        Telemetry tele = FtcDashboard.getInstance().getTelemetry();

        while (opModeInInit()) {

        }
        while (opModeIsActive()) {
            if (gamepad2.x && !xWasPressed) {
                cancelMacros();
                xWasPressed = true;
                b.setClawOpen(!b.getClawOpen());
            } else if (!gamepad2.x) xWasPressed = false;

            if (gamepad2.y && !yWasPressed) {
                cancelMacros();
                yWasPressed = true;
                b.setArmPickup(!b.getArmPickup());
            } else if (!gamepad2.y) yWasPressed = false;

            if (gamepad2.a && !aWasPressed) {
                cancelMacros();
                aMacroing = true;
                aWasPressed = true;
                b.setSlideTarget(slidePlacePos);
            } else if (!gamepad2.a) aWasPressed = false;

            if (gamepad2.b && !bWasPressed) {
                cancelMacros();
                bMacroing = true;
                bWasPressed = true;
                b.setSlideTarget(slidePickupPos);
            } else if (!gamepad2.b) bWasPressed = false;


            if (aMacroing) {
                if (abs(b.getSlidePos() - slidePlacePos) > 30) {
                    b.setArm(armPlacePos);
                    b.setTilt(tiltPlacePos);
                    b.setClawOpen(false);
                    aMacro.reset();
                }
                if (aMacro.milliseconds() > 500 && // stops resetting when we get into position
                        abs(b.getSlidePos() - slidePlacePos) < 30) { // theoretically vacuously true
                    b.setClawOpen(true);
                    aMacroing = false;
                }
            }

            if (bMacroing) {
                if (abs(b.getSlidePos() - slidePickupPos) > 30) {
                    // do something? it should automatically do what it needs to because of the failsafe.
                } else {
                    bMacroing = false;
                }
            }

            if (!(gamepad2.right_bumper && gamepad2.left_bumper)) {
                b.spintake(gamepad2.left_stick_y);
                if (gamepad2.left_stick_y == -1) { // maxed spintake -> activate B macro
                    cancelMacros(); // technically redundant
                    bMacroing = true;
                    b.setSlideTarget(slidePickupPos);
                }
                b.driveSlides(gamepad2.right_stick_y);
                b.driveHangs(gamepad2.right_trigger - gamepad2.left_trigger);
                if (gamepad2.right_bumper && gamepad2.dpad_up && !planeLaunched) {
                    b.setPlaneLaunched(true);
                    planeLaunched = true;
                    planeCooldown.reset();
                }
            } else {
                b.driveLeftHang(gamepad2.left_stick_y);
                b.driveRightHang(-gamepad2.right_stick_y);
            }

            if (planeLaunched && planeCooldown.milliseconds() > 1000) {
                b.setPlaneLaunched(false);
                planeLaunched = false;
            }
            // some edge cases, but its too clean to pass up (i tried doing max on an array, but thats not abs val)
            if (gamepad2.left_stick_y + gamepad2.right_stick_y +
                    gamepad2.left_stick_x + gamepad2.right_stick_x != 0) cancelMacros();

            b.motorDriveXYVectors(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            if (gamepad2.left_stick_y == 0) { // if player 2 not controlling intake, let player 1
                b.driveSlides(gamepad1.left_trigger - gamepad1.right_trigger);
            }
            checkCode(new Gamepad[]{gamepad1, gamepad2});
            b.tick();
        }
        }


    public void cancelMacros() {
        b.driveSlides(0);
        aMacroing = false;
        bMacroing = false;
    }

    // ignore this it dont matter ;)
    public boolean[] kKeysWerePressed = {false, false, false, false, false, false};
    public boolean addCount = false;
    public void checkCode(Gamepad[] gamepads) {
        for (Gamepad gp : gamepads) {
            if (kcount == 0 && gp.dpad_up && !kKeysWerePressed[0]) addCount = true;
            if (kcount == 1 && gp.dpad_up && !kKeysWerePressed[0]) addCount = true;
            else if (kcount == 1 && (gp.dpad_down || gp.dpad_right || gp.dpad_left || gp.a || gp.b || gp.start)) kcount = 0;
            if (kcount == 2 && gp.dpad_down && !kKeysWerePressed[1]) addCount = true;
            else if (kcount == 2 && (gp.dpad_up || gp.dpad_right || gp.dpad_left || gp.a || gp.b || gp.start)) kcount = 0;
            if (kcount == 3 && gp.dpad_down && !kKeysWerePressed[1]) addCount = true;
            else if (kcount == 3 && (gp.dpad_up || gp.dpad_right || gp.dpad_left || gp.a || gp.b || gp.start)) kcount = 0;
            if (kcount == 4 && gp.dpad_left && !kKeysWerePressed[2]) addCount = true;
            else if (kcount == 4 && (gp.dpad_up || gp.dpad_down || gp.dpad_right || gp.a || gp.b || gp.start)) kcount = 0;
            if (kcount == 5 && gp.dpad_right && !kKeysWerePressed[3]) addCount = true;
            else if (kcount == 5 && (gp.dpad_up || gp.dpad_down || gp.dpad_left || gp.a || gp.b || gp.start)) kcount = 0;
            if (kcount == 6 && gp.dpad_left && !kKeysWerePressed[2]) addCount = true;
            else if (kcount == 6 && (gp.dpad_up || gp.dpad_down || gp.dpad_right || gp.a || gp.b || gp.start)) kcount = 0;
            if (kcount == 7 && gp.dpad_right && !kKeysWerePressed[3]) addCount = true;
            else if (kcount == 7 && (gp.dpad_up || gp.dpad_down || gp.dpad_left || gp.a || gp.b || gp.start)) kcount = 0;
            if (kcount == 8 && gp.b && !kKeysWerePressed[4]) addCount = true;
            else if (kcount == 8 && (gp.dpad_up || gp.dpad_down || gp.dpad_right || gp.dpad_left || gp.a || gp.start)) kcount = 0;
            if (kcount == 9 && gp.a && !kKeysWerePressed[5]) addCount = true;
            else if (kcount == 9 && (gp.dpad_up || gp.dpad_down || gp.dpad_right || gp.dpad_left || gp.b || gp.start)) kcount = 0;
            if (kcount == 10 && gp.start && !kKeysWerePressed[6]) addCount = true;
            else if (kcount == 10 && (gp.dpad_up || gp.dpad_down || gp.dpad_right || gp.dpad_left || gp.a || gp.b)) kcount = 0;
            kKeysWerePressed = new boolean[]{gp.dpad_up, gp.dpad_down, gp.dpad_left, gp.dpad_right, gp.b, gp.a, gp.start};
            if (addCount) kcount++;
            if (kcount == 11) {
                kcount = 0;
                int startupID = hardwareMap.appContext.getResources().getIdentifier("cucumberslumber.mp3", "raw", hardwareMap.appContext.getPackageName());
                Context appContext = hardwareMap.appContext;
                SoundPlayer.getInstance().startPlaying(appContext, startupID);
            }
        }
    }
}
