package org.firstinspires.ftc.teamcode.autotests;

import static org.firstinspires.ftc.teamcode.helpers.CrazyTrajectoryGenerator.genCrazyTrajectory;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.PropAprilDet;
import org.firstinspires.ftc.teamcode.robot.boats.Bert;

@Autonomous
public class Bertauto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        double c = 0;
        int curMoveID = 0;
        boolean done = false;
        double botWidth = 16;
        Bert b = new Bert();
        b.init(hardwareMap);
        PropAprilDet ad = new PropAprilDet();
        ad.init(hardwareMap, "Front");


        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        while (opModeInInit()) {

        }
        b.rr.setPoseEstimate(new Pose2d(-36, -72+botWidth/2, PI/2));

        double prop;
        while (opModeIsActive()) {
            ad.tick();
            if (curMoveID ==0) {
                prop = ad.getProp();
                if (prop !=0) { // add a timeout or make getprop rly robust
                    if (prop == 1) {
                        b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(new Pose2d(-36, -72 + botWidth / 2, 0))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(-36, -72 + botWidth / 2, PI / 2), new Pose2d(-39, -36, 5 * PI / 6), new Pose2d(-1, 2, 0), new Pose2d(-8, 5, 0), new Pose2d(1, 1, 0), new Pose2d(1, 1, 0)))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(-39, -36, 5 * PI / 6), new Pose2d(40, -24, 0), new Pose2d(130, -120, -0.06), new Pose2d(12, -24, 0.06), new Pose2d(-2050, 2400, 0), new Pose2d(57, 0, 0.003)))
                                .build());
                    }else if(prop == 2) {
                        b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(new Pose2d(-36, -72 + botWidth / 2, 0))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(-36, -72 + botWidth / 2, PI / 2), new Pose2d(-36, -24 - botWidth / 2, PI / 2), new Pose2d(0, 12, 0), new Pose2d(0, 0, 0), new Pose2d(0, 0, 0), new Pose2d(0, 0, 0)))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(-36, -24 - botWidth / 2, PI / 2), new Pose2d(40, -36, 0), new Pose2d(0, -150, -0.05), new Pose2d(12, -90, 0.02), new Pose2d(-2000, 2200, 0), new Pose2d(57, 0, 0)))
                                .build());
                    }else {

                        b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(new Pose2d(-36, -72 + botWidth / 2, 0))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(-36, -72 + botWidth / 2, PI / 2), new Pose2d(-28, -36, PI / 6), new Pose2d(1, 2, 0), new Pose2d(8, 5, -PI / 30), new Pose2d(1, 1, 0), new Pose2d(1, 1, 0)))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(-28, -36, PI / 6), new Pose2d(40, -24, 0), new Pose2d(-120, -70, 0), new Pose2d(12, -12, 0), new Pose2d(0, 1800, 0), new Pose2d(57, 0, 0))).build());
                    }
                    done = true;
                    curMoveID++;
                }
            }
            if (curMoveID == 1) {
                b.autoTick();
            }

            if (curMoveID == 2) {

                done = false;
                ad.setWeBeProppin(false);
                if (ad.pos != null) b.rr.setPoseEstimate(new Pose2d(ad.pos.getX(), ad.pos.getY(), b.rr.getPoseEstimate().getHeading()));
                b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate()).lineTo(new Vector2d(36, -36)).build());
                curMoveID++;
            }
            b.autoTick();
            if (!b.rr.isBusy() && done) curMoveID++;
        }
    }
}
