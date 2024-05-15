package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.boats.Bert;

import java.util.Arrays;

@Config
@TeleOp
public class testop extends LinearOpMode {
    public static double extraTilt = 0.1;
    public static double counterThresh = 10;
    public static double aPos = 0.3;
    public static double tPos = 0;
    public static boolean cPos = false;
    public static boolean pPos = false;

    public static double target = 0;
    public static double SLOWDOWN = 0.6;
    public static double SLOWDOWN2 = 0.5;
    public static double SPEEDDOWN = 0.5;
    public boolean holdingPosition = false;
    public double lastSlidePos = 0;
    int pickupPos = 0;
    public boolean incrementedPickupPos = false;

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
    public boolean yMacroing = false;
    public boolean yWasPressed = false;
    public boolean aWasPressed = false;
    public boolean bWasPressed = false;
    public boolean g2wasZero = false;
    public ElapsedTime aMacro = new ElapsedTime();
    public ElapsedTime planeCooldown = new ElapsedTime();
    public boolean planeLaunched = false;
    public boolean aMacroing = false;
    public boolean bMacroing = false;
    public Bert b;
    public int kcount = 0;
    public boolean lastDirectionToggle = false;
    public double permaDirectionMultiplier = -1;
    public double tempDirectionMultiplier = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        b = new Bert();
        b.init(hardwareMap);
        b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //b.teleinit(hardwareMap);
        //PropAprilDet ad = new PropAprilDet();
        //ad.init(hardwareMap, "Front");

        Telemetry tele = FtcDashboard.getInstance().getTelemetry();

        while (opModeInInit()) {
        }
        while (opModeIsActive()) {
            try {



                if (gamepad1.left_stick_button) {
                    int[] a = b.csb.pixelBlink();
                    if (Arrays.equals(a, new int[]{0, 0})) {
                        gamepad1.rumble(300);
                    }
                }



                b.tick();
                b.csb.tick();
                //b.csbTick();
                tele.addData("AMAC", aMacroing);
                tele.addData("HPMAC", holdingPosition);
                tele.addData("BMAC", bMacroing);
                tele.addData("PMAC", planeLaunched);
                tele.update();
            } catch (Exception e) {
                tele.addData("error", e.getMessage());
                tele.log();
            }
        }
    }


    public void cancelMacros() {
        b.setSlideTargeting(false);
        aMacroing = false;
        bMacroing = false;
        yMacroing = false;
    }

}
