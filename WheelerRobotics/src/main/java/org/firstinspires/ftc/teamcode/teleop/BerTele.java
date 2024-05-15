package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.robot.boats.Bert.armPickupPos;
import static org.firstinspires.ftc.teamcode.robot.boats.Bert.leftShuv1;
import static org.firstinspires.ftc.teamcode.robot.boats.Bert.leftShuv2;
import static org.firstinspires.ftc.teamcode.robot.boats.Bert.leftShuv3;
import static org.firstinspires.ftc.teamcode.robot.boats.Bert.leftShuv4;
import static org.firstinspires.ftc.teamcode.robot.boats.Bert.leftShuv5;
import static org.firstinspires.ftc.teamcode.robot.boats.Bert.leftShuvUp;
import static org.firstinspires.ftc.teamcode.robot.boats.Bert.slidePickupPos;
import static org.firstinspires.ftc.teamcode.robot.boats.Bert.slidePlacePos;
import static org.firstinspires.ftc.teamcode.robot.boats.Bert.tiltPickupPos;
import static org.firstinspires.ftc.teamcode.robot.boats.Bert.tiltPlacePos;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.boats.Bert;

@Config
@TeleOp
public class BerTele extends LinearOpMode {
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
    public boolean lWasPressed = false;
    public boolean rWasPressed = false;
    public boolean g2wasZero = false;
    public ElapsedTime aMacro = new ElapsedTime();
    public ElapsedTime planeCooldown = new ElapsedTime();
    public boolean planeLaunched = false;
    public boolean aMacroing = false;
    public boolean lMacroing = false;
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
            b.setClawOpen(true);
            b.setArmPickup(true);
            b.setTilt(tiltPickupPos);
            b.setDownCorrection(true);
        }
        while (opModeIsActive()) {
            try {


                if (gamepad2.x && !xWasPressed) {
                    cancelMacros();
                    xWasPressed = true;
                    b.setClawOpen(!b.getClawOpen());
                } else if (!gamepad2.x) xWasPressed = false;

                if (gamepad2.right_bumper) {
                    cancelMacros();
                    if (!b.getTiltPickup()) b.setTilt(tiltPlacePos + extraTilt);
                }else if (b.getTiltPos() == tiltPlacePos + extraTilt) b.setTilt(tiltPlacePos);

                //if (gamepad2.y && !yWasPressed) {
                //    cancelMacros();
                //    yWasPressed = true;
                //    b.setArmPickup(!b.getArmPickup());
                //    b.setTiltPickup(!b.getArmPickup()); // stay same pls :)
                //} else if (!gamepad2.y) yWasPressed = false;

                // for daniel :)
                //if (gamepad1.left_bumper && gamepad1.dpad_up && !lastDirectionToggle) {
                //    lastDirectionToggle = true;
                //    permaDirectionMultiplier *= -1;
                //} else if (gamepad1.left_bumper || gamepad1.dpad_up) lastDirectionToggle = false;

                if (gamepad2.a && !aWasPressed) {
                    cancelMacros();
                    aMacroing = true;
                    aWasPressed = true;
                    b.setSlideTarget(slidePlacePos);
                } else if (!gamepad2.a) aWasPressed = false;

                //if (gamepad2.y && !yWasPressed) {
                //    cancelMacros();
                //    yWasPressed = true;
                //    yMacroing = true;
                //    b.setSlideTarget(slidePickupPos);
                //}else if (!gamepad2.y) yWasPressed = false;

                if (gamepad2.b && !bWasPressed) {
                    cancelMacros();
                    bMacroing = true;
                    bWasPressed = true;
                    b.setSlideTarget(slidePickupPos);
                } else if (!gamepad2.b) bWasPressed = false;


                if (yMacroing) {
                    b.setClawOpenUNSAFE(false);
                    if (abs(b.getSlidePos() - lastSlidePos) <= 8) { // done moving MIGHT BREAK SHIT
                        b.resetSlides();
                        b.setClawOpenUNSAFE(true);
                        yMacroing = false;
                    }
                    lastSlidePos = b.getSlidePos();
                    if (abs(b.getSlidePos() - slidePickupPos) > 30) {
                        // do something? it should automatically do what it needs to because of the failsafe.
                    } else {
                        yMacroing = false;
                        b.setSlideTargeting(false);
                    }
                }



                if (aMacroing) {
                    if (abs(b.getSlidePos() - slidePlacePos) > 30) {
                        b.setClawOpen(false);
                        aMacro.reset();
                    }
                    if (abs(b.getSlidePos() - slidePlacePos) < 200) {
                        b.setArmPickup(false);
                        b.setTiltPickup(false);
                    }
                    if (aMacro.milliseconds() > 500 && // stops resetting when we get into position
                            abs(b.getSlidePos() - slidePlacePos) < 30) { // theoretically vacuously true
                        aMacroing = false;
                    }
                }

                if (bMacroing) {
                    if (abs(b.getSlidePos() - slidePickupPos) > 30) {
                        b.setArm(armPickupPos);
                        b.setTilt(tiltPickupPos);
                        // do something? it should automatically do what it needs to because of the failsafe.
                       // if (abs(b.getSlidePos() - lastSlidePos) <= 2) {
                       //     b.resetSlides();
                       //     bMacroing = false;
                       // }
                        //lastSlidePos = b.getSlidePos();
                    }else if(abs(b.getSlidePos() - slidePickupPos) < 300){
                        b.setClawOpen(true);

                    }else {
                        bMacroing = false;
                        b.setSlideTargeting(false);
                    }
                }
                if (gamepad2.dpad_down) { // just dpad down
                    b.resetSlides();
                }
                if (!(gamepad2.right_bumper && gamepad2.left_bumper)) {
                    b.spintake(-gamepad2.left_stick_y);
                    if (gamepad2.left_stick_y == -1) { // maxed spintake -> activate B macro BAD IDEA
                        //cancelMacros(); // technically redundant
                        //bMacroing = true;
                        //b.setSlideTarget(slidePickupPos);
                    }
                    if ((!aMacroing && !bMacroing && !yMacroing) && gamepad2.right_stick_y == 0 && !holdingPosition) {
                        b.setSlideTarget(b.getSlidePos());
                        holdingPosition = true;
                    }else if ((!aMacroing && !bMacroing && !yMacroing) || gamepad2.right_stick_y != 0) {
                        cancelMacros();
                        b.driveSlides(-gamepad2.right_stick_y);
                    }
                    if (holdingPosition) b.setSlideTargeting(true);
                    if (gamepad2.right_stick_y != 0) holdingPosition = false;
                    //b.tick();
                    b.driveHangs(gamepad2.right_trigger - gamepad2.left_trigger);
                    if (gamepad2.right_bumper && gamepad2.dpad_up && !planeLaunched) {
                        b.setPlaneLaunched(true);
                        planeLaunched = true;
                        planeCooldown.reset();
                    }
                    if (gamepad1.right_bumper && gamepad1.dpad_up && !planeLaunched) {
                        b.setPlaneLaunched(true);
                        planeLaunched = true;
                        planeCooldown.reset();
                    }
                } else {
                    b.driveLeftHang(gamepad2.left_stick_y);
                    b.driveRightHang(gamepad2.right_stick_y);
                }

                if (planeLaunched && planeCooldown.milliseconds() > 1000) {
                    b.setPlaneLaunched(false);
                    planeLaunched = false;
                }
                // some edge cases, but its too clean to pass up (i tried doing max on an array, but thats not abs val)
                if (gamepad2.left_stick_y + gamepad2.right_stick_y +
                        gamepad2.left_stick_x + gamepad2.right_stick_x != 0) cancelMacros();

                if (gamepad1.left_bumper) tempDirectionMultiplier = SPEEDDOWN;
                else tempDirectionMultiplier = 1;

                b.motorDriveXYVectors(tempDirectionMultiplier * permaDirectionMultiplier * gamepad1.left_stick_x, tempDirectionMultiplier * permaDirectionMultiplier * -gamepad1.left_stick_y, (gamepad1.right_bumper ? 1 : SLOWDOWN) * (gamepad1.left_bumper ? SLOWDOWN2 : 1) * gamepad1.right_stick_x);
                //b.fieldCentricDrive(directionMultiplier * gamepad1.left_stick_x, directionMultiplier * -gamepad1.left_stick_y, (gamepad1.right_bumper ? 1 : SLOWDOWN) * gamepad1.right_stick_x);
                if (gamepad2.left_stick_y == 0 && gamepad1.a) { // if player 2 not controlling intake, let player 1
                    b.spintake(-1);
                }
                if (gamepad2.left_stick_y == 0 && gamepad1.x) { // if player 2 not controlling intake, let player 1
                    b.spintake(0.6);
                }
                if (gamepad2.left_stick_y == 0 && gamepad1.y) { // if player 2 not controlling intake, let player 1
                    b.spintake(1);
                }


                if (gamepad1.left_stick_button || gamepad1.dpad_left) {
                    int[] a = b.csb.pixelBlink();
                }
                b.csb.tick();

                if (lMacroing) {
                    if (abs(b.getSlidePos() - slidePickupPos) > 30) {
                        b.setArm(armPickupPos);
                        b.setTilt(tiltPickupPos);
                        // do something? it should automatically do what it needs to because of the failsafe.
                        // if (abs(b.getSlidePos() - lastSlidePos) <= 2) {
                        //     b.resetSlides();
                        //     bMacroing = false;
                        // }
                        //lastSlidePos = b.getSlidePos();
                    } else {
                        b.setClawOpen(true);
                        lMacroing = false;
                        b.setSlideTargeting(false);
                    }
                }
                b.setRunToBottom(gamepad2.right_stick_button);
                if (gamepad1.left_trigger == 0) {
                    if (gamepad1.right_trigger > 0.99 && !incrementedPickupPos) {
                        incrementedPickupPos = true;
                        pickupPos += 1;
                    } else if (gamepad1.right_trigger <= 0.01) {
                        pickupPos = 0;
                    } else if (gamepad1.right_trigger < 0.99 && incrementedPickupPos) {
                        incrementedPickupPos = false;
                    }
                    b.setLeftShuv(pickupPos == 0 ? leftShuvUp : (pickupPos == 1 ? leftShuv5 : (pickupPos == 2 ? leftShuv4 : (pickupPos == 3 ? leftShuv3 : (pickupPos == 4 ? leftShuv2 : leftShuv1)))));

                }else {
                    b.setLeftShuv(leftShuv1);
                }


                b.tick();
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
        lMacroing = false;
    }

}
