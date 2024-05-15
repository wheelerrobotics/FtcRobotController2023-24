package org.firstinspires.ftc.teamcode.autotests.main;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.helpers.PropAprilDet;
import org.firstinspires.ftc.teamcode.robot.boats.Bert;

//GOOD THEORETCALLY
@Autonomous
public class BASE_BlueBackdropMiddle extends LinearOpMode {
    public int localizationCount = 0;
    ElapsedTime cooldown;
    int curMoveID = 0;
    boolean firstTimeSlides = true;
    boolean localizing = false;
    double prop = 0;
    Bert b = null;
    PropAprilDet ad = null;
    Telemetry tele = null;
    boolean done = true;
    ElapsedTime clawTimer = null;
    boolean clawTiming = false;
    boolean relocalized = false;
    ElapsedTime localizationWait;
    int[] states;


    @Override
    public void runOpMode() throws InterruptedException {
        double c = 0;
        double botWidth = 16;
        DriveConstants.MAX_VEL = 50;
        DriveConstants.MAX_ACCEL = 35;
        localizationWait = new ElapsedTime();
        b = new Bert();
        b.setCawtFailsafe(false);
        b.init(hardwareMap);
        ad = new PropAprilDet();
        ad.init(hardwareMap, "Front", false);
        cooldown = new ElapsedTime();
        clawTimer = new ElapsedTime();
        tele = FtcDashboard.getInstance().getTelemetry();
        b.rr.setPoseEstimate(new Pose2d(12, 65, Math.toRadians(-90.00)));
        while (opModeInInit()) {

            b.rr.setPoseEstimate(new Pose2d(12, 65, Math.toRadians(-90.00)));
            if (ad.bv.opened) {
                ad.setWeBeProppin(true);
                ad.tick();
                prop = ad.getProp();
            }
        }

        while (opModeIsActive()) {
            if (curMoveID ==0) {
                if (done) {
                    done = false;

                    ad.setWeBeProppin(true);
                    cooldown.reset();
                    while (cooldown.milliseconds() < 1000) {
                        ad.tick();
                        prop = ad.getProp();
                    }
                    //prop = (prop == 1 ? 3 : (prop == 2) ? 1 : 2);

                    if (prop != 0) { // add a timeout or make getprop rly robust
                        b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())
                                //.splineTo(new Vector2d(-47.00, -40.67), Math.toRadians(120.00)) //#1 spline
                                //.splineTo(new Vector2d(-36.44, -36.27), Math.toRadians(89.02)) //#2 spline
                                .addTemporalMarker(0, ()->{
                                    b.setArmPickup(true);
                                    b.setTiltPickup(true);
                                    b.setClawOpen(true);
                                })
                                //oth V
                                .splineTo(prop == 3 ? new Vector2d(-24+30.74, 38.38) : (prop == 2 ? new Vector2d(-24+36.44, 36.27) : new Vector2d(-24+44.50, 39.00)), prop == 3 ? Math.toRadians(-120.00) : (prop == 2 ? Math.toRadians(-90.00) : Math.toRadians(-60.00))) //#3 spline

                                .setReversed(true)
                                .splineTo(new Vector2d(24, 60), Math.toRadians(90))
                                .setReversed(false)
                                .splineTo(new Vector2d(38, 40.79), Math.toRadians(0))
                                .addTemporalMarker(0, ()->{
                                    b.setArmPickup(true);
                                    b.setTiltPickup(true);
                                    b.setClawOpen(true);
                                })
                                .addSpatialMarker(new Vector2d(37, 43), ()->{
                                    //ad.setWeBeProppin(false);
                                    localizing = true;
                                    localizationWait.reset();
                                })
                                .addTemporalMarker(2, ()->{
                                    b.setSlideTarget(400);
                                    b.setClawOpen(false);
                                    // up up
                                })
                                .addTemporalMarker(3, ()->{
                                    //b.setTiltPickup(false);
                                    b.setTilt(Bert.tiltPlacePos + 0.04);
                                    b.setArmPickup(false);
                                    // up up
                                })
                                .setReversed(false)
                                .waitSeconds(0.5)
                                //.splineTo(new Vector2d(34, -36.97), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(10, PI, 15), SampleMecanumDrive.getAccelerationConstraint(10))
                                // MAYBE PUT BACK LATER IDK .waitSeconds(0.5)
                                // maybe make prev traj tinier and then do a waitSec call? shouldnt mess up displacement marker, hmm...

                                //.splineTo(new Vector2d(36, -36), Math.toRadians(0.00))
                                //.waitSeconds(10)
                                .splineToConstantHeading(new Vector2d(53, prop == 3 ? 34 : (prop == 2 ? 39 : 46)), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(20, PI, 15), SampleMecanumDrive.getAccelerationConstraint(20))
                                .addDisplacementMarker(()->{
                                    b.setClawOpen(true);
                                    b.setSlideTarget(800);
                                    // drop
                                })

                                .addTemporalMarker(pathTime -> pathTime - 0.7, ()->{
                                    b.setSlideTarget(-80);
                                    //b.setDownCorrection(true);
                                    b.setClawOpen(true);
                                    b.setTiltPickup(true);
                                    b.setArmPickup(true);
                                })
                                .setReversed(true)

                                .splineToConstantHeading(new Vector2d(56, 14), Math.toRadians(0.00))
                                .addTemporalMarker(pathTime -> pathTime, increment)
                                .build());

                    }
                }

            }
            if (curMoveID == 1) return;
            b.autoTick();
            if (localizing && localizationWait.milliseconds() < 2000 && localizationWait.milliseconds() > 1000) {
                // still gotta figure out what tags to use
                tele.addData("localizing", true);
                if (localizationCount >=5) {
                    tele.addData("counted", true);
                }else tele.addData("counted", false);
                ad.getDetected();
                if (ad.pos != null && localizationCount < 5) {
                    if (abs((ad.pos.getX() - 5.8) - b.rr.getPoseEstimate().getX()) < 8 && abs((ad.pos.getY() + 6.4) - b.rr.getPoseEstimate().getY()) < 8) {
                        tele.addData("apx", ad.pos.getX() - 3.875);
                        tele.addData("apy", ad.pos.getY() - 8);

                        b.rr.setPoseEstimate(new Pose2d(ad.pos.getX() - 5.8, ad.pos.getY() + 6.4, ad.pos.getHeading()));
                        relocalized = true;
                        localizationCount+=1;
                    }
                }
            }else {

                tele.addData("localizing", false);
            }
            if (clawTiming && clawTimer.milliseconds() > 85) {
                b.setClawOpen(!b.getClawOpen());
                clawTiming = false;
            }
            tele.addData("curmove", curMoveID);
            tele.addData("relocalized", relocalized);

            tele.addData("busy", b.rr.isBusy());
            tele.update();
            //if (!b.rr.isBusy()) done = true;

            if (done) {
                //done = false;
                curMoveID++;
                tele.addData("curmove", curMoveID);
                tele.update();
            }
        }
    }
    public void relocalize(Pose2d target, int count) {
        localizationCount = 0;
        while (localizationCount < count) {
            ad.setWeBeProppin(false);
            ad.getDetected(); // update

            // offsets are for when rotation is 0
            if (ad.pos != null) {
                tele.addData("tagx", ad.pos.getX() - 6.875);
                tele.addData("tagy", ad.pos.getY() - 6); // flipped for red :)
                tele.update();
            }
            if (ad.pos != null)
                b.rr.setPoseEstimate(new Pose2d(ad.pos.getX() - 5.8, ad.pos.getY() - 3.9, b.rr.getPoseEstimate().getHeading()));
            b.rr.followTrajectorySequence(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate()).lineToLinearHeading(target).build());
            localizationCount++;
        }
        localizationCount = 0;
        return;
    }
    public MarkerCallback increment = () ->{ done = true; };
}