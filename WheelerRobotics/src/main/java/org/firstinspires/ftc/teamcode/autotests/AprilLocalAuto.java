package org.firstinspires.ftc.teamcode.autotests;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.demos.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.robot.boats.Odo;
@Disabled
@Config
public class AprilLocalAuto extends LinearOpMode {
    AprilTagLocalizer atl = null;
    Odo o = null;
    public static int moveId = 0;
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

        while (opModeInInit()){
            tryRelocalize();
        };
        o.rr.setPoseEstimate(new Pose2d(12, 60, -PI/2));
        tele.addData("XYR", o.rr.getPoseEstimate().getX() + "  " + o.rr.getPoseEstimate().getY() + "  " + o.rr.getPoseEstimate().getHeading());

        int c = 0;
        boolean done = false;
        boolean local = false;
        while (opModeIsActive()) {
            c++;
            tele.addData("Hung?", c);
            tele.update();
            //tryRelocalize();
            tele.addData("X", o.rr.getPoseEstimate().getX());
            tele.addData("Y", o.rr.getPoseEstimate().getY());
            tele.addData("R", o.rr.getPoseEstimate().getHeading());
            tele.update();
            o.rr.update();
            if (!o.rr.isBusy() && moveId == 0) {
                o.rr.followTrajectorySequenceAsync(o.rr.trajectorySequenceBuilder(o.rr.getPoseEstimate()).lineToLinearHeading(new Pose2d(12, 36, -PI/2)).lineToLinearHeading(new Pose2d(15, 12, -PI/2)).build());
                local = false;
            }
            /*if (!o.rr.isBusy() && moveId == 1) {
                o.rr.followTrajectorySequenceAsync(o.rr.trajectorySequenceBuilder(o.rr.getPoseEstimate()).lineToLinearHeading(new Pose2d(38, 36, 0)).build());
                local = false;
            }
            if (!o.rr.isBusy() && moveId == 2) {
                o.rr.followTrajectorySequenceAsync(o.rr.trajectorySequenceBuilder(o.rr.getPoseEstimate()).lineToLinearHeading(new Pose2d(50, 36, 0), SampleMecanumDrive.getVelocityConstraint(10, 1, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(20)).build());
                local = true;
            }
            if (!o.rr.isBusy() && moveId == 3) {
                o.rr.followTrajectorySequenceAsync(o.rr.trajectorySequenceBuilder(o.rr.getPoseEstimate()).lineToLinearHeading(new Pose2d(36, 36, 0), SampleMecanumDrive.getVelocityConstraint(10, 1, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(20)).build());
                local = true;
            }
            if (!o.rr.isBusy() && moveId == 4) {
                o.rr.followTrajectorySequenceAsync(o.rr.trajectorySequenceBuilder(o.rr.getPoseEstimate()).lineToLinearHeading(new Pose2d(36, 12, 0)).lineToLinearHeading(new Pose2d(-12, 12, 0)).turn(4*PI).lineToLinearHeading(new Pose2d(36, 12, 0)).lineToLinearHeading(new Pose2d(38, 36, 0)).build());
                local = false;
            }
            if (!o.rr.isBusy() && moveId == 5) {
                o.rr.followTrajectorySequenceAsync(o.rr.trajectorySequenceBuilder(o.rr.getPoseEstimate()).lineToLinearHeading(new Pose2d(50, 36, 0), SampleMecanumDrive.getVelocityConstraint(10, 1, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(20)).build());
                local = true;
            }
            if (!o.rr.isBusy() && moveId == 6) {
                o.rr.followTrajectorySequenceAsync(o.rr.trajectorySequenceBuilder(o.rr.getPoseEstimate()).lineToLinearHeading(new Pose2d(38, 36, 0), SampleMecanumDrive.getVelocityConstraint(10, 1, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(20)).build());
                local = true;
            }
            if (!o.rr.isBusy() && moveId == 7) {
                o.rr.followTrajectorySequenceAsync(o.rr.trajectorySequenceBuilder(o.rr.getPoseEstimate()).lineToLinearHeading(new Pose2d(38, 54)).lineToLinearHeading(new Pose2d(56, 54, 0)).build());
                local = false;
            }
            if (!o.rr.isBusy() && moveId == 8) {
                done = true;
            }*/
            if (c% 10 == 0 && local) tryRelocalize();
            if (!o.rr.isBusy() && done) break;
            if (!o.rr.isBusy()) moveId++;
        }


    }
    public void tryRelocalize() {
        Pose2d pose = atl.tryLocalize();

        if (pose != null) o.rr.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), o.rr.getPoseEstimate().getHeading()));

    }
}
