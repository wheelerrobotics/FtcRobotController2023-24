package org.firstinspires.ftc.teamcode.autotests;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.demos.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.robot.boats.Odo;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AprilLocalAuto extends LinearOpMode {
    AprilTagLocalizer atl = null;
    Odo o = null;
    @Override
    public void runOpMode() throws InterruptedException {
        o = new Odo();
        o.init(hardwareMap);
        o.autoinit();
        atl = new AprilTagLocalizer();
        atl.init(hardwareMap);
        //      ___
        //     /   \   berd
        //     \˚∆˚/
        //      / \
        //     /| |\
        //    /-M M-\
        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry tele = dash.getTelemetry();
        tele.addData("atl", atl);
        tele.addData("ad", atl.ad);
        tele.addData("atl", atl.ad.bv);
        tele.addData("atl", atl.ad.bv.webcam);


        waitForStart();
        TrajectorySequence traj = null;
        TrajectorySequence traj2 = null;
        o.rr.followTrajectorySequenceAsync(o.rr.trajectorySequenceBuilder(new Pose2d(12, 12, -PI/2)).lineToLinearHeading(new Pose2d(36, 36, -PI/2)).build());
        int c = 0;
        while (opModeIsActive()) {
            c++;
            tele.addData("Hung?", c);
            tele.update();
            o.rr.update();
            tryRelocalize();
        }

    }
    public void tryRelocalize() {
        Pose2d pose = atl.tryLocalize();
        if (pose != null) o.rr.setPoseEstimate(pose);
    }
}
