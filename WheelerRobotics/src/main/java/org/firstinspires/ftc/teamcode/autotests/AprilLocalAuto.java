package org.firstinspires.ftc.teamcode.autotests;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.demos.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.robot.boats.Odo;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class AprilLocalAuto extends LinearOpMode {
    AprilTagLocalizer atl = null;
    Odo o = null;
    @Override
    public void runOpMode() throws InterruptedException {
        o = new Odo();
        o.init(hardwareMap);
        o.autoinit();
        atl.init(hardwareMap);
        //      ___
        //     /   \   berd
        //     \˚∆˚/
        //      / \
        //     /| |\
        //    /-M M-\
        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry tele = dash.getTelemetry();


        waitForStart();
        TrajectorySequence traj = null;
        TrajectorySequence traj2 = null;
        o.rr.followTrajectorySequenceAsync(o.rr.trajectorySequenceBuilder(new Pose2d(-12, 12, PI/2)).lineToLinearHeading(new Pose2d(-36, 36, PI/2)).build());

        while (opModeIsActive()) {
            tryRelocalize();
            o.rr.update();
        }

    }
    public void tryRelocalize() {
        o.rr.setPoseEstimate(atl.tryLocalize());
    }
}
