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
public class RedBackdropNew2 extends LinearOpMode {
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
        ad.init(hardwareMap, "Back", true);
        cooldown = new ElapsedTime();
        clawTimer = new ElapsedTime();
        tele = FtcDashboard.getInstance().getTelemetry();
        b.rr.setPoseEstimate(new Pose2d(-36, -65, Math.toRadians(90.00)));
        while (opModeInInit()) {

            b.rr.setPoseEstimate(new Pose2d(-36, -65, Math.toRadians(90.00)));
            if (ad.bv.opened) {
                ad.setWeBeProppin(true);
                ad.tick();
            }
        }

        while (opModeIsActive()) {
            if (curMoveID ==0) {
                if (done) {
                    done = false;

                    ad.setWeBeProppin(true);
                    cooldown.reset();
                    while (cooldown.milliseconds() < 500) {
                        ad.tick();
                        prop = ad.getProp();
                    }
                    ad.setWeBeProppin(false);
                    prop = (prop == 1 ? 1 : (prop == 2) ? 3 : 2);

                    if (prop != 0) { // add a timeout or make getprop rly robust
                            b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())
                                    //.splineTo(new Vector2d(-47.00, -40.67), Math.toRadians(120.00)) //#1 spline
                                    //.splineTo(new Vector2d(-36.44, -36.27), Math.toRadians(89.02)) //#2 spline
                                    .addTemporalMarker(0, ()->{
                                        b.setArmPickup(true);
                                        b.setTiltPickup(true);
                                        b.setClawOpen(true);
                                    })
                                    .splineTo(prop == 3 ? new Vector2d(-32.74, -38.38) : (prop == 2 ? new Vector2d(-36.44, -36.27) : new Vector2d(-44.50, -39.00)), prop == 3 ? Math.toRadians(60.00) : (prop == 2 ? Math.toRadians(90.00) : Math.toRadians(120.00))) //#3 spline
                                    .setReversed(true)
                                    .splineToSplineHeading(new Pose2d(-45, -50, Math.toRadians(-20)), Math.toRadians(170), SampleMecanumDrive.getVelocityConstraint(40, PI/2, 15), SampleMecanumDrive.getAccelerationConstraint(30))
                                    .addTemporalMarker(0.5, () ->{
                                        b.spintake(-1);
                                        b.setLeftShuv(Bert.leftShuv5);
                                        // down
                                    })
                                    .addTemporalMarker(pathTime -> pathTime-0.05, increment)
                                    .splineToConstantHeading(new Vector2d(-52, -45.00), Math.toRadians(170))
                                    .waitSeconds(0.6)
                                    .addTemporalMarker(pathTime -> pathTime-0.05, ()->{
                                        b.setLeftShuv(Bert.leftShuvUp);
                                        // up
                                    })

                                    // *****************************************************************************************

                                    //.addTemporalMarker(pathTime -> pathTime, increment)
                                    /*.setReversed(true)
                                    .splineTo(new Vector2d(47.53, -37.14), Math.toRadians(180.0))
                                    .splineToConstantHeading(new Vector2d(24.82, -60.73), Math.toRadians(180.00))
                                    .splineTo(new Vector2d(-12.50, -60.38), Math.toRadians(180.00))
                                    .splineTo(new Vector2d(-55.38, -47.25), Math.toRadians(150.00))
                                    .setReversed(false)
                                    .splineTo(new Vector2d(-12.50, -61.38), Math.toRadians(0.00))
                                    .splineTo(new Vector2d(53.69, -36.97), Math.toRadians(0.00))*/
                                    .build());

                    }
                }

            }
            if (curMoveID ==1) {
                if (done) {
                    done = false;

                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())
                            .addTemporalMarker(1.2, ()->{
                                b.setLeftShuv(Bert.leftShuvUp);
                                b.spintake(0.75);
                                // up
                            })
                            // *****************************************************************************************

                            .addTemporalMarker(2.5, ()->{
                                b.setLeftShuv(Bert.leftShuvUp);
                                b.spintake(0);
                                // up
                            })
                            .addTemporalMarker(4, ()->{
                                b.setSlideTarget(600);
                                b.setClawOpen(false);
                                // up up
                            })
                            .addTemporalMarker(4.6, ()->{
                                //b.setTiltPickup(false);
                                b.setTilt(Bert.tiltPlacePos + 0.04);
                                b.setArmPickup(false);
                                // up up
                            })
                            .back(2)
                            .setReversed(false)

                            .splineToSplineHeading(new Pose2d(-49, -46, 0), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(-36, -61.38), Math.toRadians(0))
                            .splineToSplineHeading(new Pose2d(24.17, -61.00, 0), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(40, PI/2, 15), SampleMecanumDrive.getAccelerationConstraint(40))

                            .addSpatialMarker(new Vector2d(37, -43), ()->{
                                localizing = true;
                                localizationWait.reset();
                            })
                            .splineToConstantHeading(new Vector2d(37, -40), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(40, PI/2, 15), SampleMecanumDrive.getAccelerationConstraint(40))


                            // slowly move forward while localizing ( essentially a wait )
                            // slowly move forward while localizing ( essentially a wait )
                             .setReversed(false)
                             .waitSeconds(0.5)
                            //.splineTo(new Vector2d(34, -36.97), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(10, PI, 15), SampleMecanumDrive.getAccelerationConstraint(10))
                            // MAYBE PUT BACK LATER IDK .waitSeconds(0.5)
                            // maybe make prev traj tinier and then do a waitSec call? shouldnt mess up displacement marker, hmm...

                            //.splineTo(new Vector2d(36, -36), Math.toRadians(0.00))
                            //.waitSeconds(10)
                            .splineToConstantHeading(new Vector2d(54, prop == 2 ? -40 : (prop == 3 ? -32 : -40)), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(20, PI, 15), SampleMecanumDrive.getAccelerationConstraint(20))
                            .addDisplacementMarker(()->{
                                b.setClawOpen(true);
                                b.setSlideTarget(1100);
                                clawTiming = true; // set timer for quick flip back
                                clawTimer.reset();
                                // drop
                            })
                            // i should split into seperate traj seqs
                            // .addTemporalMarker(()->{
                            //    b.setClawOpen(false);
                            // catch
                            //}) // gonna be tough to time esp w/ changes :/
                            .setReversed(true)
                            .splineToConstantHeading(new Vector2d(50, prop == 3 ? -42 : (prop == 2 ? -36 : -30)), Math.toRadians(0.00))
                            .splineToConstantHeading(new Vector2d(53, prop == 3 ? -42 : (prop == 2 ? -38 : -30)), Math.toRadians(0.00), SampleMecanumDrive.getVelocityConstraint(20, PI, 15), SampleMecanumDrive.getAccelerationConstraint(20))

                            .addTemporalMarker(pathTime -> pathTime-0.05, ()->{
                                b.setClawOpen(true);
                                b.setSlideTarget(1200);
                                // drop
                            })
                            .addTemporalMarker(pathTime -> pathTime, increment)
                            //.addTemporalMarker(pathTime -> pathTime, increment)
                            /*.setReversed(true)
                            .splineTo(new Vector2d(47.53, -37.14), Math.toRadians(180.0))
                            .splineToConstantHeading(new Vector2d(24.82, -60.73), Math.toRadians(180.00))
                            .splineTo(new Vector2d(-12.50, -60.38), Math.toRadians(180.00))
                            .splineTo(new Vector2d(-55.38, -47.25), Math.toRadians(150.00))
                            .setReversed(false)
                            .splineTo(new Vector2d(-12.50, -61.38), Math.toRadians(0.00))
                            .splineTo(new Vector2d(53.69, -36.97), Math.toRadians(0.00))*/
                            .build());


                }

            }
            if (curMoveID == 2) {
                if (done){
                    done = false;
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())
                            .setReversed(true)
                            .addTemporalMarker(0.8, ()->{
                                b.setSlideTarget(-80);
                                b.setDownCorrection(true);
                                b.setClawOpen(true);
                                b.setTiltPickup(true);
                                b.setArmPickup(true);
                            })
                            .setReversed(true)
                            .splineToConstantHeading(new Vector2d(24.82, -55.5), Math.toRadians(180.00))
                            .splineToSplineHeading(new Pose2d(-12, -56, 0), Math.toRadians(180))
                            .splineToSplineHeading(new Pose2d(-36.50, -56.5, 0), Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(-54.50, -37.00), Math.toRadians(160))
                            .addTemporalMarker(pathTime -> pathTime - 3, () ->{
                                b.spintake(-1);
                                b.setLeftShuv(Bert.leftShuv4);
                                // down
                            })
                            .addTemporalMarker(pathTime -> pathTime, increment)
                            .build());
                }
            }
            if (curMoveID == 3) {
                if (done){
                    done = false;
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())
                            .waitSeconds(1.4)
                            // *****************************************************************************************
                            .addTemporalMarker(0.5, () ->{
                                b.setLeftShuv(Bert.leftShuv3);
                                b.spintake(-1);
                                // down
                            })
                            .addTemporalMarker(1.6, ()->{
                                b.setLeftShuv(Bert.leftShuvUp);
                                b.spintake(0.75);
                                // up
                            })
                            .addTemporalMarker(3.9, ()->{
                                b.setLeftShuv(Bert.leftShuvUp);
                                b.spintake(0);
                                // up
                            })
                            .addTemporalMarker(4.0, ()->{
                                b.setSlideTarget(1200);
                                b.setClawOpen(false);
                                // up up
                            })
                            .addTemporalMarker(4.7, ()->{
                                //b.setTiltPickup(false);
                                b.setTilt(Bert.tiltPlacePos + 0.04);
                                b.setArmPickup(false);
                                // up up
                            })
                            .setReversed(false)
                            .splineToConstantHeading(new Vector2d(-36, -58.38), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(24.17, -58.00), Math.toRadians(0.00))
                            //.splineToConstantHeading(new Vector2d(40, -41), Math.toRadians(0.00))

                            //.lineToConstantHeading(new Vector2d(36, -36))
                            //.waitSeconds(5)
                            .splineToConstantHeading(new Vector2d(52, -36), Math.toRadians(0.00))
                            .addTemporalMarker(pathTime -> pathTime-0.31, ()->{
                                b.setClawOpen(true);
                                b.setSlideTarget(1600);
                                done = true;
                                // drop
                            })
                                    .waitSeconds(0.3)
                            .build());
                }
            }
            if (curMoveID == 4) {
                if (done){
                    done = false;
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())
                            // same thing as above then park
                            .addTemporalMarker(0.8, ()->{
                                b.setSlideTarget(-80);
                                b.setDownCorrection(true);
                                b.setClawOpen(true);
                                b.setTiltPickup(true);
                                b.setArmPickup(true);
                            })
                            .setReversed(true)
                            .splineToConstantHeading(new Vector2d(54, -63), Math.toRadians(0.00))
                            .build());
                }
            }
            /*if (curMoveID == 1) {
                relocalize(new Pose2d(36, -44, 0), 2);
                done = true;
            }

            if (curMoveID == 2) {
                if (done){
                    done = false;
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(new Pose2d(36, -44, 0))
                            .waitSeconds(0.4)
                            .lineToLinearHeading(new Pose2d(57, prop == 1 ? -46 : (prop == 2 ? -34 :  -26), 0))
                            .waitSeconds(0.8)
                            .addTemporalMarker(0, () -> {
                                b.setSlideTarget(600);
                            })
                            .addTemporalMarker(0.1, () -> {
                                b.setClawOpen(false);
                            })
                            .addTemporalMarker(0.4, () -> {
                                b.setTilt(tiltPlacePos + (prop == 1 ? 0.02 : (prop == 2 ? 0.02 : 0.02)));
                                b.setArmPickup(false);
                            })
                            .addTemporalMarker(2, () -> {
                                b.setClawOpen(true);
                            })
                            .addTemporalMarker(2.2, () -> {
                                b.setSlideTarget(1000);
                            })
                            .addTemporalMarker(2.3, increment)
                            .build());
                }
            }
            if (curMoveID == 3) {
                if (done){
                    done = false;
                    Pose2d curpos = new Pose2d(57, -36, 0);//prop == 1 ? new Pose2d(54, 44, 0) : (prop == 2 ? new Pose2d(54, 36, 0) : new Pose2d(54, 28, 0));
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(curpos)
                            .back(5)
                            .strafeRight(40)
                            .forward(10)
                            .waitSeconds(2)
                            .addTemporalMarker(0.5, () -> {
                                b.setClawOpen(true);
                                b.setArmPickup(true);
                                b.setTiltPickup(true);
                                b.setLeftShuv(leftShuvUp);
                            })
                            .addTemporalMarker(1, () -> {
                                b.setDownCorrection(true);
                                b.setDownCorrectionFactor(0.1);
                                b.setSlideTarget(0);
                            })
                            .addTemporalMarker(6, increment)

                            .build());
                }
            }
            if (curMoveID == 4) {
                return;
            }*/
            b.autoTick();
            if (localizing && localizationWait.milliseconds() < 2000 && localizationWait.milliseconds() > 1000) {
                // still gotta figure out what tags to use
                tele.addData("localizing", true);
                if (localizationCount >=5) {
                    tele.addData("counted", true);
                }else tele.addData("counted", false);
                ad.getDetected();
                if (ad.pos != null && localizationCount < 5) {
                    if (abs((ad.pos.getX() - 5.8) - b.rr.getPoseEstimate().getX()) < 8 && abs((ad.pos.getY() - 3.9) - b.rr.getPoseEstimate().getY()) < 8) {
                        tele.addData("apx", ad.pos.getX() - 3.875);
                        tele.addData("apy", ad.pos.getY() - 8);

                        b.rr.setPoseEstimate(new Pose2d(ad.pos.getX() - 5.8, ad.pos.getY() - 3.9, ad.pos.getHeading()));
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
