package org.firstinspires.ftc.teamcode.autotests.main;

import static org.firstinspires.ftc.teamcode.helpers.CrazyTrajectoryGenerator.genCrazyTrajectory;
import static org.firstinspires.ftc.teamcode.robot.boats.Bert.leftShuvDown;
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
public class BlueBackdropPlus extends LinearOpMode {
    public int localizationCount = 0;
    ElapsedTime cooldown;
    int curMoveID = 0;
    boolean firstTimeSlides = true;
    double prop = 0;
    Bert b = null;
    PropAprilDet ad = null;
    Telemetry tele = null;
    boolean done = false;


    @Override
    public void runOpMode() throws InterruptedException {
        double c = 0;
        double botWidth = 16;
        b = new Bert();
        b.setCawtFailsafe(false);
        b.init(hardwareMap);
        ad = new PropAprilDet();
        ad.init(hardwareMap, "Front", false);
        cooldown = new ElapsedTime();
        tele = FtcDashboard.getInstance().getTelemetry();
        while (opModeInInit()) {

            b.rr.setPoseEstimate(new Pose2d(12, 64, -PI/2));
            if (ad.bv.opened) {
                ad.setWeBeProppin(true);
                ad.tick();
            }
        }

        while (opModeIsActive()) {

            if (curMoveID ==0) {
                if (!b.rr.isBusy()) {
                    ad.setWeBeProppin(true);
                    cooldown.reset();
                    while (cooldown.milliseconds() < 2000) {
                        ad.tick();
                    }
                    ad.tick();
                    prop = ad.getProp();
                    if (prop != 0) { // add a timeout or make getprop rly robust
                        if (prop == 1) {
                            b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())
                                    .addTrajectory(genCrazyTrajectory(new Pose2d(12, 64, -PI / 2), new Pose2d(14, 39, -PI / 6), new Pose2d(1, -2, 0), new Pose2d(8, -5, 0), new Pose2d(-1, -1, 0), new Pose2d(-1, -1, 0)))
                                    .addTrajectory(genCrazyTrajectory(new Pose2d(15, 36, -PI / 6), new Pose2d(36, 44, 0), new Pose2d(-13, 12, -0.06), new Pose2d(0, -1, 0.06), new Pose2d(0, 1000, 0), new Pose2d(57, 0, 0.003)))
                                    .build());
                        } else if (prop == 2) {
                            b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())

                                    .addTrajectory(genCrazyTrajectory(new Pose2d(12, 64, -PI / 2), new Pose2d(12, 38, -PI / 2), new Pose2d(0, -2, 0), new Pose2d(0, -5, 0), new Pose2d(1, 1, 0), new Pose2d(1, 1, 0)))
                                    .addTrajectory(genCrazyTrajectory(new Pose2d(12, 35, -PI / 2), new Pose2d(36, 44, 0), new Pose2d(0, 13, 0), new Pose2d(12, -12, 0), new Pose2d(0, 200, 0), new Pose2d(57, 0, 0)))
                                    .build());
                        } else {
                            b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())
                                    .addTrajectory(genCrazyTrajectory(new Pose2d(12, 64, -PI / 2), new Pose2d(12, 36, 7 * PI / 6), new Pose2d(-1, -2, 0.01), new Pose2d(-8, -5, -.06), new Pose2d(-1, -1, 0), new Pose2d(-1, -1, 0)))
                                    .addTrajectory(genCrazyTrajectory(new Pose2d(8, 36, 7 * PI / 6), new Pose2d(36, 44, 0), new Pose2d(13, 12, 0.1), new Pose2d(0, -1, 0.06), new Pose2d(0, 1000, 0), new Pose2d(57, 0, 0.003)))
                                    .build());
                        }

                    }
                }

            }

            if (curMoveID == 1) {
                relocalize(new Pose2d(36, 44, 0), 4);
                done = true;
            }
            if (curMoveID == 2) {
                if (!b.rr.isBusy()){
                    done = false;
                    b.setSlideTarget(700);
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(new Pose2d(36, 44, 0))
                            .lineToLinearHeading(new Pose2d(50, prop == 1 ? 46 : (prop == 2 ? 34 :  26), 0))
                                    .lineToLinearHeading(new Pose2d(40, prop == 1 ? 46 : (prop == 2 ? 34 :  26), 0))
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
                            .build());
                }
            }
            if (curMoveID == 3) {
                if (done){
                    done = false;
                    Pose2d curpos = prop == 1 ? new Pose2d(54, 44, 0) : (prop == 2 ? new Pose2d(54, 36, 0) : new Pose2d(54, 28, 0));
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(curpos)
                            .addTemporalMarker(1, () -> {
                                b.setClawOpen(true);
                                b.setArmPickup(true);
                                b.setTiltPickup(true);
                                b.setSlideTarget(-10);
                                b.spintake(-1);
                            })
                            .addTrajectory(genCrazyTrajectory(curpos, new Pose2d(-50, 16, 0), new Pose2d(-100, -10, 0), new Pose2d(40, 110, 0), new Pose2d(1000,-1200 + ((prop - 1) * 300),0), new Pose2d(1000,1400, 0)))

                            .addTrajectory(genCrazyTrajectory(new Pose2d(-50, 16, 0), new Pose2d(36, 44, 0), new Pose2d(260, -20, 0), new Pose2d(0, 100, 0), new Pose2d(100,-200,0), new Pose2d(0,100, 0)))


                            .addSpatialMarker(new Vector2d(-50, 0), ()->{
                                b.setLeftShuv(leftShuvDown);
                            }) // shuv down
                            .addSpatialMarker(new Vector2d(-62, 5), ()->{
                                //b.spintake(-1);
                            }) // intake on
                            .addSpatialMarker(new Vector2d(-60, 20), ()->{
                                //b.spintake(-0.5);
                            }) // intake reverse
                            .addSpatialMarker(new Vector2d(-50, 20), ()->{
                                //b.spintake(0);
                                //b.setLeftShuv(leftShuvUp);
                            }) // intake off & shuv up
                            .build());
                }
            }
            if (curMoveID == 4) {
                relocalize(new Pose2d(36, 44, 0), 4);
                done = true;
            }
            if (curMoveID == 5) {
                if (!b.rr.isBusy()) done = true;
                if (!b.rr.isBusy()){
                    done = false;
                    b.setSlideTarget(700);
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(new Pose2d(36, 44, 0))
                            .lineToLinearHeading(new Pose2d(57, 34, 0))
                            .waitSeconds(0.9)
                            .addTemporalMarker(0, () -> {
                                b.setSlideTarget(700);
                            })
                            .addTemporalMarker(1, () -> {
                                b.setClawOpen(false);
                            })
                            .addTemporalMarker(1.5, () -> {
                                b.setTilt(tiltPlacePos + (prop == 1 ? 0.02 : (prop == 2 ? 0.02 : 0.02)));
                                b.setArmPickup(false);
                            })
                            .addTemporalMarker(2.2, () -> {
                                b.setClawOpen(true);
                            })
                            .addTemporalMarker(2.4, () -> {
                                b.setSlideTarget(1000);
                            })
                            .build());
                }
            }
            if (curMoveID == 6) {
                if (!b.rr.isBusy()){
                    done = false;
                    Pose2d curpos = new Pose2d(57, 36, 0);//prop == 1 ? new Pose2d(54, 44, 0) : (prop == 2 ? new Pose2d(54, 36, 0) : new Pose2d(54, 28, 0));
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(curpos)
                            .lineTo(new Vector2d(44, curpos.getY()))
                            .strafeLeft(prop == 1 ? 20 : (prop == 2 ? 30 : 40))
                            .forward(10)
                            .addTemporalMarker(0.5, () -> {
                                b.setClawOpen(true);
                                b.setArmPickup(true);
                                b.setTiltPickup(true);
                            })
                            .addTemporalMarker(2, () -> {
                                b.setDownCorrection(true);
                                b.setDownCorrectionFactor(0.1);
                                b.setSlideTarget(0);
                            })
                            .build());
                }
            }
            if (curMoveID == 7) {
                return;
            }
            b.autoTick();
            tele.addData("curmove", curMoveID);
            tele.addData("busy", b.rr.isBusy());
            tele.update();
            if (!b.rr.isBusy()) done = true;
            if (done) {
                done = false;
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
                tele.addData("tagy", ad.pos.getY() + 6);
                tele.update();
            }
            if (ad.pos != null)
                b.rr.setPoseEstimate(new Pose2d(ad.pos.getX() - 7.875, ad.pos.getY() + 7, b.rr.getPoseEstimate().getHeading()));
            b.rr.followTrajectorySequence(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate()).lineToLinearHeading(target).build());
            //curMoveID++;
            localizationCount++;
        }
        localizationCount = 0;
        return;
    }
}
