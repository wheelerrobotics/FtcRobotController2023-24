package org.firstinspires.ftc.teamcode.autotests.main;

import static org.firstinspires.ftc.teamcode.helpers.CrazyTrajectoryGenerator.genCrazyTrajectory;
import static org.firstinspires.ftc.teamcode.robot.boats.Bert.tiltPlacePos;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.PropAprilDet;
import org.firstinspires.ftc.teamcode.robot.boats.Bert;

@Autonomous
public class BlueStacks extends LinearOpMode {
    public int localizationCount = 0;
    ElapsedTime cooldown;
    int curMoveID = 0;
    boolean firstTimeSlides = true;
    double prop = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        double c = 0;
        boolean done = false;
        double botWidth = 16;
        Bert b = new Bert();
        b.setCawtFailsafe(false);
        b.init(hardwareMap);
        PropAprilDet ad = new PropAprilDet();
        ad.init(hardwareMap, "Front", false);
        cooldown = new ElapsedTime();

        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        while (opModeInInit()) {

            b.rr.setPoseEstimate(new Pose2d(-36, 64, -PI/2));
            if (ad.bv.opened) {
                ad.setWeBeProppin(true);
                ad.tick();
            }
        }

        while (opModeIsActive()) {
            if (curMoveID ==0) {
                ad.setWeBeProppin(true);
                cooldown.reset();
                while (cooldown.milliseconds() < 2000) {
                    ad.tick();
                }
                prop = ad.getProp();
                if (prop !=0) { // add a timeout or make getprop rly robust
                    if (prop == 1) {
                        b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(new Pose2d(-36, 64, -PI/2))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(-36, 64, -PI/2), new Pose2d(-32,36, -PI/6), new Pose2d(1, -2, 0), new Pose2d(8,-5, 0), new Pose2d(-1,-1,0), new Pose2d(-1,-1, 0)))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(-33, 36, -PI / 6), new Pose2d(36, 40, 0), new Pose2d(-120, 60, 0), new Pose2d(0, 90, 0), new Pose2d(200, -1800, 0), new Pose2d(1000, 200, 0)))
                                .build());
                    }else if(prop == 2) {
                        b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(new Pose2d(-36, 64, -PI/2))
                                .lineToLinearHeading(new Pose2d(-36,34, -PI/2))
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(-36, 65, -PI / 2), new Pose2d(-36, 32, -PI / 2), new Pose2d(0.1, 0, 0), new Pose2d(-0.1, 0, 0), new Pose2d(0, 0, 0), new Pose2d(0, 0, 0)))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(-36, 32, -PI / 2), new Pose2d(36, 40, 0), new Pose2d(0, 150, 0.05), new Pose2d(12, 90, -0.02), new Pose2d(-2000, -2700, 0), new Pose2d(570, -200, 0)))
                                .build());
                    }else {

                        b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(new Pose2d(-36, 64, -PI/2))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(-36, 64, -PI/2), new Pose2d(-33,36, -5*PI/6), new Pose2d(-1, -2, 0.0), new Pose2d(-8,-5, 0.06), new Pose2d(-1,-1,0), new Pose2d(-1,-1, 0)))

                                //.addTrajectory(genCrazyTrajectory(new Pose2d(-36, 64, -PI / 2), new Pose2d(-37, 36,  -5*PI / 6), new Pose2d(-1, -2, 0), new Pose2d(-8, -5, 0), new Pose2d(1, -1, 0), new Pose2d(1, -1, 0)))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(-37, 36,  -5*PI / 6), new Pose2d(36, 40, 0), new Pose2d(130, 100, 0.06), new Pose2d(0, 24, -0.06), new Pose2d(-2350, -2500, 0), new Pose2d(1007, -100, -0.003)))
                                .build());
                    }
                    done = true;
                    curMoveID++;
                }
            }
            if (curMoveID == 1) {

            }

            if (curMoveID == 2) {
                done = false;
                ad.setWeBeProppin(false);
                ad.getDetected(); // update

                // offsets are for when rotation is 0
                if (ad.pos != null) {
                    tele.addData("tagx", ad.pos.getX() - 6.875);
                    tele.addData("tagy", ad.pos.getY() + 6);
                    tele.update();
                }
                if (ad.pos != null) b.rr.setPoseEstimate(new Pose2d(ad.pos.getX()-7.875, ad.pos.getY()+7, b.rr.getPoseEstimate().getHeading()));
                b.rr.followTrajectorySequence(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate()).lineToLinearHeading(new Pose2d(36, 44, 0)).build());
                //curMoveID++;
                localizationCount++;
                if (localizationCount == 4) curMoveID++;
                done = true;
            }
            if (curMoveID == 3) {
                if (!b.rr.isBusy()){
                    b.setSlideTarget(900);
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(new Pose2d(36, 44, 0))
                            .lineToLinearHeading(new Pose2d(57, prop == 1 ? 46 : (prop == 2 ? 34 :  26), 0))
                            .waitSeconds(0.9)
                            .addTemporalMarker(0, () -> {
                                b.setSlideTarget(700);
                            })
                            .addTemporalMarker(0.3, () -> {
                                b.setClawOpen(false);
                            })
                            .addTemporalMarker(0.8, () -> {
                                b.setTilt(tiltPlacePos + (prop == 1 ? 0.02 : (prop == 2 ? 0.02 : 0.02)));
                                b.setArmPickup(false);
                            })
                            .addTemporalMarker(1.5, () -> {
                                b.setClawOpen(true);
                            })
                            .addTemporalMarker(2, () -> {
                                b.setSlideTarget(1000);
                            })
                            .addTemporalMarker(2, () ->{
                                curMoveID++;
                            })
                            .build());
                }
            }
            if (curMoveID == 5) {
                if (!b.rr.isBusy()){
                    Pose2d curpos = prop == 1 ? new Pose2d(57, 44, 0) : (prop == 2 ? new Pose2d(57, 36, 0) : new Pose2d(57, 28, 0));
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(curpos)
                            .lineTo(new Vector2d(44, curpos.getY()))
                            .waitSeconds(3)
                            .addTemporalMarker(0.5, () -> {
                                b.setClawOpen(true);
                                b.setArmPickup(true);
                                b.setTiltPickup(true);
                            })
                            .addTemporalMarker(1, () -> {
                                b.setDownCorrection(true);
                                b.setDownCorrectionFactor(0.1);
                                b.setSlideTarget(0);
                            })
                            .build());
                }
            }
            if (curMoveID == 6) {
                return;
            }
            b.autoTick();
            tele.addData("curmove", curMoveID);
            tele.update();
            if (!b.rr.isBusy() && done) curMoveID++;
        }
    }
}

