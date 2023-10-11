package org.firstinspires.ftc.teamcode.autotests;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.AprilDet2;
import org.firstinspires.ftc.teamcode.robot.boats.Odo;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp
@Config
public class TagFollowerOpMode extends LinearOpMode {
    public static double targetThresh = 0.1;
    public static double targetX = -1;
    public static double targetZ = 2;
    public static double targetID = 1 ;
    public static double zCoefficient = -1;
    public static double xCoefficient = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        Odo o = new Odo();
        o.init(hardwareMap);
        o.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        o.autoinit();
        //    ___
        //   /   \   berd
        //   \˚∆˚/
        //    / \
        //   /| |\
        //  /-M M-\
        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry tele = dash.getTelemetry();
        AprilDet2 ad = new AprilDet2();
        ad.init(hardwareMap);


        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> a = ad.getDetections();
            AprilTagDetection tag = null;
            if (a != null) {
                if (a.size() > 0) {
                    for (AprilTagDetection d : a) {
                        if (d.id == targetID) {
                            tag = d;
                            break;
                        }
                    }
                    if (tag == null || (abs(tag.rawPose.x-targetX) < targetThresh && abs(tag.rawPose.z-targetZ) < targetThresh)) {
                        //o.motorStop();

                        continue;
                    }

                    TrajectorySequence t = o.rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0)).lineToConstantHeading(new Vector2d((tag.rawPose.x-targetX)*xCoefficient, (tag.rawPose.z-targetZ)*zCoefficient)).build();
                    o.rr.followTrajectorySequence(t);
                    // with this math, itll approach from an angle because x depends partly on z,
                    // because the farther away the tag is, the closer to the middle it will be because camera perspective
                    // (the angle will match the fov degrees of the camera)
                    tele.addData("x", tag.rawPose.x);
                    tele.addData("z", tag.rawPose.z);
                    tele.addData("id", tag.id);
                    tele.update();
                }
            }
        }
    }
}
