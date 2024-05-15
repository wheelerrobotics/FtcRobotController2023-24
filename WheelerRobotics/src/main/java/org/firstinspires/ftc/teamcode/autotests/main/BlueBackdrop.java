package org.firstinspires.ftc.teamcode.autotests.main;

import static org.firstinspires.ftc.teamcode.autotests.Auto.incrementer;
import static org.firstinspires.ftc.teamcode.helpers.CrazyTrajectoryGenerator.genCrazyTrajectory;
import static org.firstinspires.ftc.teamcode.helpers.CrazyTrajectoryGenerator.genCrazyTrajectoryConstrained;
import static org.firstinspires.ftc.teamcode.robot.boats.Bert.leftShuvUp;
import static org.firstinspires.ftc.teamcode.robot.boats.Bert.tiltPlacePos;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.PropAprilDet;
import org.firstinspires.ftc.teamcode.robot.boats.Bert;

//GOOD THEORETCALLY
@Autonomous
public class BlueBackdrop extends LinearOpMode {
    public int localizationCount = 0;
    ElapsedTime cooldown;
    int curMoveID = 0;
    boolean firstTimeSlides = true;
    double prop = 0;
    Bert b = null;
    PropAprilDet ad = null;
    Telemetry tele = null;
    boolean done = true;
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
                if (done) {
                    done = false;
                    ad.setWeBeProppin(true);
                    cooldown.reset();
                    while (cooldown.milliseconds() < 500) {
                        ad.tick();
                    }
                    ad.tick();
                    prop = ad.getProp();
                    if (prop != 0) { // add a timeout or make getprop rly robust
                        if (prop == 1) {
                            b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())
                                    .addTrajectory(genCrazyTrajectoryConstrained(new Pose2d(12, 64, -PI / 2), new Pose2d(17, 39, -PI / 6), new Pose2d(1, -2, 0), new Pose2d(8, -5, 0), new Pose2d(-1, -1, 0), new Pose2d(-1, -1, 0), 30, 20))
                                    .addTrajectory(genCrazyTrajectory(new Pose2d(15, 36, -PI / 6), new Pose2d(36, 44, 0), new Pose2d(-13, 12, -0.06), new Pose2d(0, -1, 0.06), new Pose2d(0, 1000, 0), new Pose2d(57, 0, 0.003)))
                                    .addTrajectory(incrementer(b, increment))

                                    .build());
                        } else if (prop == 2) {
                            b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())

                                    .addTrajectory(genCrazyTrajectoryConstrained(new Pose2d(12, 64, -PI / 2), new Pose2d(12, 38, -PI / 2), new Pose2d(0, -2, 0), new Pose2d(0, -2, 0), new Pose2d(1, 1, 0), new Pose2d(1, 1, 0), 30, 20))
                                    .addTrajectory(genCrazyTrajectory(new Pose2d(12, 35, -PI / 2), new Pose2d(36, 44, 0), new Pose2d(0, 13, 0), new Pose2d(12, -12, 0), new Pose2d(0, 200, 0), new Pose2d(57, 0, 0)))
                                    .addTrajectory(incrementer(b, increment))

                                    .build());
                        } else {
                            b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())
                                    .addTrajectory(genCrazyTrajectoryConstrained(new Pose2d(12, 64, -PI / 2), new Pose2d(11, 36, 7 * PI / 6), new Pose2d(-1, -2, 0.01), new Pose2d(-8, -5, -.06), new Pose2d(-1, -1, 0), new Pose2d(-1, -1, 0), 30, 20))
                                    .addTrajectory(genCrazyTrajectory(new Pose2d(8, 36, 7 * PI / 6), new Pose2d(36, 44, 0), new Pose2d(13, 12, 0.1), new Pose2d(0, -1, 0.06), new Pose2d(0, 1000, 0), new Pose2d(57, 0, 0.003)))
                                    .addTrajectory(incrementer(b, increment))

                                    .build());
                        }

                    }
                }

            }

            if (curMoveID == 1) {
                relocalize(new Pose2d(36, 44, 0), 2);
                done = true;
            }
            if (curMoveID == 2) {
                if (done){
                    done = false;

                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(new Pose2d(36, 44, 0))
                            .waitSeconds(0.4)
                            .lineToLinearHeading(new Pose2d(57, prop == 1 ? 46 : (prop == 2 ? 34 :  29), 0))
                            .waitSeconds(0.9)
                            .addTemporalMarker(0, () -> {
                                b.setSlideTarget(700);
                            })
                            .addTemporalMarker(0.1, () -> {
                                b.setClawOpen(false);
                            })
                            .addTemporalMarker(0.4, () -> {
                                b.setTilt(tiltPlacePos + (prop == 1 ? 0.02 : (prop == 2 ? 0.02 : 0.02)));
                                b.setArmPickup(false);
                            })
                            .addTemporalMarker(1.7, () -> {
                                b.setClawOpen(true);
                            })
                            .addTemporalMarker(2, () -> {
                                b.setSlideTarget(1000);
                            })
                            .addTemporalMarker(2.1, increment)
                            .build());
                }
            }
            if (curMoveID == 3) {
                if (done){
                    done = false;
                    Pose2d curpos = new Pose2d(57, 36, 0);//prop == 1 ? new Pose2d(54, 44, 0) : (prop == 2 ? new Pose2d(54, 36, 0) : new Pose2d(54, 28, 0));
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(curpos)
                            .back(10)
                            .strafeLeft(40)
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
            }
            b.autoTick();
            tele.addData("curmove", curMoveID);
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
                tele.addData("tagy", ad.pos.getY() + 6);
                tele.update();
            }
            if (ad.pos != null)
                b.rr.setPoseEstimate(new Pose2d(ad.pos.getX() - 7.875, ad.pos.getY() + 7, b.rr.getPoseEstimate().getHeading()));
            b.rr.followTrajectorySequence(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate()).lineToLinearHeading(target).build());
            localizationCount++;
        }
        localizationCount = 0;
        return;
    }
    public MarkerCallback increment = () ->{ done = true; };
}
