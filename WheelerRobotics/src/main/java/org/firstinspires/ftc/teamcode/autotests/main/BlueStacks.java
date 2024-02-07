package org.firstinspires.ftc.teamcode.autotests.main;

import static org.firstinspires.ftc.teamcode.autotests.Auto.getPropPos;
import static org.firstinspires.ftc.teamcode.autotests.Auto.incrementer;
import static org.firstinspires.ftc.teamcode.autotests.Auto.relocalize;
import static org.firstinspires.ftc.teamcode.helpers.CrazyTrajectoryGenerator.genCrazyTrajectoryConstrained;
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
public class BlueStacks extends LinearOpMode {
    public int localizationCount = 0;
    ElapsedTime cooldown;
    int curMoveID = 0;
    boolean firstTimeSlides = true;

    double prop = 3;
    boolean done = true;
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

            b.rr.setPoseEstimate(new Pose2d(-36, 64, -PI/2));
            if (ad.bv.opened) {
                ad.setWeBeProppin(true);
                ad.tick();
            }
        }

        while (opModeIsActive()) {
            if (curMoveID ==0 && done) {
                b.rr.setPoseEstimate(new Pose2d(-36, 64, -PI/2));
                done = false;
                prop = getPropPos(ad, 2000, 2);
                if (prop == 1) {
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())
                            .addTrajectory(genCrazyTrajectoryConstrained(new Pose2d(-36, 64, -PI/2), new Pose2d(-33.5,36, -PI/6), new Pose2d(1, -2, 0), new Pose2d(8,-5, 0), new Pose2d(-1,-1,0), new Pose2d(-1,-1, 0), 30, 20))
                            .addTrajectory(genCrazyTrajectoryConstrained(new Pose2d(-33, 36, -PI/6), new Pose2d(36, 44, 0), new Pose2d(-80, 40, 0.03), new Pose2d(0, 90, 0), new Pose2d(-400, -1800, -0.001), new Pose2d(1500, 500, 0), 30, 20))
                            .addTrajectory(incrementer(b, increment))

                            .build());
                }else if(prop == 2) {
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())

                            .lineToLinearHeading(new Pose2d(-36,34, -PI/2))
                            //.addTrajectory(genCrazyTrajectory(new Pose2d(-36, 65, -PI / 2), new Pose2d(-36, 32, -PI / 2), new Pose2d(0.1, 0, 0), new Pose2d(-0.1, 0, 0), new Pose2d(0, 0, 0), new Pose2d(0, 0, 0)))
                            .addTrajectory(genCrazyTrajectoryConstrained(new Pose2d(-36, 32, -PI / 2), new Pose2d(36, 48, 0), new Pose2d(0, 150, 0.05), new Pose2d(12, 190, -0.02), new Pose2d(-2300, -2800, 0), new Pose2d(1200, 1500, 0), 30, 20))
                            .addTrajectory(incrementer(b, increment))
                            .build());
                }else {
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())
                            .addTrajectory(genCrazyTrajectoryConstrained(new Pose2d(-36, 64, -PI/2), new Pose2d(-38,36, -5*PI/6), new Pose2d(-1, -2, 0.0), new Pose2d(-8,-5, 0), new Pose2d(-1,-1,0), new Pose2d(-1,-1, 0), 30, 20))

                            //.addTrajectory(genCrazyTrajectory(new Pose2d(-36, 64, -PI / 2), new Pose2d(-37, 36,  -5*PI / 6), new Pose2d(-1, -2, 0), new Pose2d(-8, -5, 0), new Pose2d(1, -1, 0), new Pose2d(1, -1, 0)))
                            .addTrajectory(genCrazyTrajectoryConstrained(new Pose2d(-39, 36,  -5*PI / 6), new Pose2d(36, 44, 0), new Pose2d(130, 100, 0.05), new Pose2d(0, 24, -0.06), new Pose2d(-2350, -2500, 0), new Pose2d(800, -100, -0.003), 30, 20))
                            .addTrajectory(incrementer(b, increment))

                            .build());
                }
                // INCREMENT ONCE PATH IS RUN
            }
            if (curMoveID == 1 && done) {
               // b.rr.setPoseEstimate(new Pose2d(36, 40, 0)); //FOR TESTING TAKE OUT IN REAL
                done = false; // redundant, but here to stay consistent
                relocalize(b, ad, new Pose2d(36, 44, 0), 4);
                done = true;
            }
            if (curMoveID == 3 && done) {
                done = false;
                b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(new Pose2d(36, 44, 0))
                        .waitSeconds(0.3)
                        .lineToLinearHeading(new Pose2d(52, prop == 1 ? 42 : (prop == 2 ? 36 :  28), 0))
                        .waitSeconds(0.8)
                        .addTemporalMarker(0, () -> {
                            b.setSlideTarget(800);
                        })
                        .addTemporalMarker(0.4, () -> {
                            b.setClawOpen(false);
                        })
                        .addTemporalMarker(0.6, () -> {
                            b.setTilt(tiltPlacePos + 0.06);
                            b.setArmPickup(false);
                        })
                        .addTemporalMarker(1.8, () -> {
                            b.setClawOpen(true);
                        })
                        .addTemporalMarker(1.9, () -> {
                            b.setSlideTarget(1000);
                        })
                        .addTemporalMarker(2.2, increment)
                        .build());
                // INCREMENT ONCE PATH IS RUN
            }
            if (curMoveID == 4 && done) {
                done = false;
                Pose2d curpos = prop == 1 ? new Pose2d(54, 44, 0) : (prop == 2 ? new Pose2d(54, 36, 0) : new Pose2d(54, 28, 0));
                b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(curpos)
                        .lineTo(new Vector2d(44, curpos.getY()))
                        .waitSeconds(3)
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
                        .addTemporalMarker(2.5, increment)
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

