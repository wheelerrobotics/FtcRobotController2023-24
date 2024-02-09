package org.firstinspires.ftc.teamcode.autotests.main;

import static org.firstinspires.ftc.teamcode.autotests.Auto.getPropPos;
import static org.firstinspires.ftc.teamcode.autotests.Auto.incrementer;
import static org.firstinspires.ftc.teamcode.autotests.Auto.placerBlueLow;
import static org.firstinspires.ftc.teamcode.autotests.Auto.relocalizeB;
import static org.firstinspires.ftc.teamcode.helpers.CrazyTrajectoryGenerator.genCrazyTrajectory;
import static org.firstinspires.ftc.teamcode.helpers.CrazyTrajectoryGenerator.genCrazyTrajectoryConstrained;
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

//GOOD LN
@Autonomous
public class BlueStacks636 extends LinearOpMode {
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

            b.rr.setPoseEstimate(new Pose2d(-36, 64, -PI / 2));
            if (ad.bv.opened) {
                ad.setWeBeProppin(true);
                ad.tick();
            }
        }

        while (opModeIsActive()) {
            if (curMoveID == 0 && done) {
                b.rr.setPoseEstimate(new Pose2d(-36, 64, -PI / 2));
                done = false;
                prop = getPropPos(ad, 2000, 2);
                if (prop == 1) {
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())
                            .addTrajectory(genCrazyTrajectoryConstrained(new Pose2d(-36, 64, -PI/2), new Pose2d(-32,36, -PI/6), new Pose2d(1, -2, 0), new Pose2d(8,-5, 0), new Pose2d(-1,-1,0), new Pose2d(-1,-1, 0), 30, 20))
                            //.addTrajectory(genCrazyTrajectory(new Pose2d(-33, 36, -PI / 6), new Pose2d(36, 40, 0), new Pose2d(-120, 60, 0), new Pose2d(0, 90, 0), new Pose2d(200, -1800, 0), new Pose2d(1000, 200, 0)))
                                    .addTrajectory(genCrazyTrajectory(new Pose2d(-33, 36, -PI / 6), new Pose2d(-36, 36, 0), new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d()))
                            .lineToLinearHeading(new Pose2d(-36, 54, 0))
                            .addTrajectory(genCrazyTrajectoryConstrained(new Pose2d(-36, 54, 0), new Pose2d(36, 40, 0), new Pose2d(0, 30, 0), new Pose2d(0, -30, 0), new Pose2d(0, 0, 0), new Pose2d(500, 300, 0), 30, 30))
                            .addTrajectory(incrementer(b, increment))

                            .build());
                } else if (prop == 2) {
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())

                            .lineToLinearHeading(new Pose2d(-36,34, -PI/2))
                            //.addTrajectory(genCrazyTrajectory(new Pose2d(-36, 65, -PI / 2), new Pose2d(-36, 32, -PI / 2), new Pose2d(0.1, 0, 0), new Pose2d(-0.1, 0, 0), new Pose2d(0, 0, 0), new Pose2d(0, 0, 0)))
                            //.addTrajectory(genCrazyTrajectory(new Pose2d(-36, 32, -PI / 2), new Pose2d(36, 40, 0), new Pose2d(0, 150, 0.05), new Pose2d(12, 90, -0.02), new Pose2d(-2000, -2700, 0), new Pose2d(570, -200, 0)))
                            .back(3)
                            .lineToLinearHeading(new Pose2d(-36, 54, 0))
                            .addTrajectory(genCrazyTrajectoryConstrained(new Pose2d(-36, 54, 0), new Pose2d(36, 40, 0), new Pose2d(0, 30, 0), new Pose2d(0, -30, 0), new Pose2d(0, 0, 0), new Pose2d(500, 300, 0), 30, 30))
                            .addTrajectory(incrementer(b, increment))
                            .build());
                } else {
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())
                            .addTrajectory(genCrazyTrajectoryConstrained(new Pose2d(-36, 64, -PI/2), new Pose2d(-38,36, -5*PI/6), new Pose2d(-1, -2, 0.0), new Pose2d(-8,-5, 0), new Pose2d(-1,-1,0), new Pose2d(-1,-1, 0), 30, 20))
                            //.addTrajectory(genCrazyTrajectory(new Pose2d(-37, 36,  -5*PI / 6), new Pose2d(36, 40, 0), new Pose2d(130, 100, 0.06), new Pose2d(0, 24, -0.06), new Pose2d(-2350, -2400, 0), new Pose2d(1007, -100, -0.003)))
                            .addTrajectory(genCrazyTrajectory(new Pose2d(-33, 36, -PI / 6), new Pose2d(-36, 50, 0), new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d()))
                            .lineToLinearHeading(new Pose2d(-36, 54, 0))
                            .addTrajectory(genCrazyTrajectoryConstrained(new Pose2d(-36, 54, 0), new Pose2d(36, 40, 0), new Pose2d(0, 30, 0), new Pose2d(0, -30, 0), new Pose2d(0, 0, 0), new Pose2d(500, 300, 0), 30, 30))
                            .addTrajectory(incrementer(b, increment))

                            .build());
                }
                // INCREMENT ONCE PATH IS RUN
            }
            if (curMoveID == 1 && done) {
                // b.rr.setPoseEstimate(new Pose2d(36, 40, 0)); //FOR TESTING TAKE OUT IN REAL
                done = false; // redundant, but here to stay consistent
                relocalizeB(b, ad, new Pose2d(36, 44, 0), 4);
                done = true;
            }
            if (curMoveID == 3 && done) {
                done = false;
                b.rr.followTrajectorySequenceAsync(placerBlueLow(b, prop, increment));
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

    public MarkerCallback increment = () -> {
        done = true;
    };
}


