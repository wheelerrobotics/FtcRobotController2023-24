package org.firstinspires.ftc.teamcode.autotests.main;

import static org.firstinspires.ftc.teamcode.autotests.Auto.getPropPos;
import static org.firstinspires.ftc.teamcode.autotests.Auto.incrementer;
import static org.firstinspires.ftc.teamcode.autotests.Auto.relocalizeB;
import static org.firstinspires.ftc.teamcode.helpers.CrazyTrajectoryGenerator.genCrazyTrajectory;
import static org.firstinspires.ftc.teamcode.robot.boats.Bert.tiltPlacePos;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.PropAprilDet;
import org.firstinspires.ftc.teamcode.robot.boats.Bert;

@Autonomous
public class tuna extends LinearOpMode {
    public int localizationCount = 0;
    ElapsedTime cooldown;
    int curMoveID = 0;
    boolean done = true;
    boolean firstTimeSlides = true;
    double prop = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        double c = 0;
        double botWidth = 16;
        Bert b = new Bert();
        b.setCawtFailsafe(false);
        b.init(hardwareMap);
        PropAprilDet ad = new PropAprilDet();
        ad.init(hardwareMap, "Front", false);
        cooldown = new ElapsedTime();

        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        while (opModeInInit()) {

            b.rr.setPoseEstimate(new Pose2d(12, 64, -PI/2));
            if (ad.bv.opened) {
                ad.setWeBeProppin(true);
                ad.tick();
            }
        }

        while (opModeIsActive()) {
            if (curMoveID ==0 && done) {
                done = false;
                prop = getPropPos(ad, 2000, 2);
                if (prop == 1) {
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())
                            .addTrajectory(genCrazyTrajectory(new Pose2d(12, 64, -PI/2), new Pose2d(14,39, -PI/6), new Pose2d(1, -2, 0), new Pose2d(8,-5, 0), new Pose2d(-1,-1,0), new Pose2d(-1,-1, 0)))
                            .addTrajectory(genCrazyTrajectory(new Pose2d(15,36, -PI/6), new Pose2d(36, 44, 0), new Pose2d(-13, 12, -0.06), new Pose2d(0, -1, 0.06), new Pose2d(0,1000,0), new Pose2d(57,0, 0.003)))
                            .addTrajectory(incrementer(b, increment))

                            .build());
                }else if(prop == 2) {
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())

                            .addTrajectory(genCrazyTrajectory(new Pose2d(12, 64, -PI/2), new Pose2d(12,38, -PI/2), new Pose2d(0, -2, 0), new Pose2d( 0,-5, 0), new Pose2d(1,1,0), new Pose2d(1,1, 0)))
                            .addTrajectory(genCrazyTrajectory(new Pose2d(12,35, -PI/2), new Pose2d(36, 44, 0), new Pose2d(0, 13, 0), new Pose2d(12, -12, 0), new Pose2d(0,200,0), new Pose2d(57,0, 0)))
                            .addTrajectory(incrementer(b, increment))
                            .build());
                }else {
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())
                            .addTrajectory(genCrazyTrajectory(new Pose2d(12, 64, -PI/2), new Pose2d(12,36, 7*PI/6), new Pose2d(-1, -2, 0.01), new Pose2d(-8,-5, -.06), new Pose2d(-1,-1,0), new Pose2d(-1,-1, 0)))
                            .addTrajectory(genCrazyTrajectory(new Pose2d(8,36, 7*PI/6), new Pose2d(36, 44, 0), new Pose2d(13, 12, 0.1), new Pose2d(0, -1, 0.06), new Pose2d(0,1000,0), new Pose2d(57,0, 0.003)))
                            .addTrajectory(incrementer(b, increment))

                            .build());
                }
                // INCREMENT ONCE PATH IS RUN
            }
            if (curMoveID == 1 && done) {
                done = false; // redundant, but here to stay consistent
                relocalizeB(b, ad, new Pose2d(36, 44, 0), 2);
                done = true;
            }
            if (curMoveID == 3 && done) {
                done = false;
                b.setSlideTarget(700);
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
                        .addTemporalMarker(1, 0, increment)
                        .build());
                // INCREMENT ONCE PATH IS RUN
            }
            if (curMoveID == 4 && done) {
                done = false;
                Pose2d curpos = prop == 1 ? new Pose2d(54, 44, 0) : (prop == 2 ? new Pose2d(54, 36, 0) : new Pose2d(54, 28, 0));
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
                        .addTemporalMarker(1, 0, increment)
                        .build());
                // INCREMENT ONCE PATH IS RUN
            }
            if (curMoveID == 6) {
                return;
            }
            b.autoTick();
            tele.addData("curmove", curMoveID);
            tele.update();
            if (done) curMoveID++;
        }
    }
    public MarkerCallback increment = () ->{ done = true; };

}
