package org.firstinspires.ftc.teamcode.autotests;

import static java.lang.Math.abs;
import static java.lang.Math.asin;
import static java.lang.Math.atan2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.teamcode.helpers.AprilDet2;
import org.firstinspires.ftc.teamcode.robot.boats.Odo;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp
@Config
public class TagFollowerOpModeRR extends LinearOpMode {
    public static double targetThresh = 0.3;
    public static double targetX = 0;
    public static double targetZ = 130;
    public static double targetID = 1;
    public static double zCoefficient = 0.03;
    public static double xCoefficient = -0.07;
    public static double rCoefficient = -1;
    public static double xOffset = 6;
    public static double yOffset = -1;
    public static double wiggleMag = 0.4;
    public static double wiggleThresh = 40;
    public static double pitchThresh = 0.05;
    public static double pTarget = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Odo o = new Odo();
        o.init(hardwareMap);
        //o.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
        boolean positioned = false;
        int wiggle = 0;

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

                    if (tag== null) {
                        wiggle++;
                        tele.addData("wiggle", wiggle);
                        tele.update();
                        continue;
                    }
                    MatrixF rot = tag.rawPose.R;
                    double yaw = atan2(rot.get(1, 0),rot.get(0,0));
                    double roll = atan2(rot.get(2, 1),rot.get(2,2));
                    double pitch = -asin(rot.get(2, 0)); // THIS ONE
                    if (tag == null || (abs(tag.rawPose.x-targetX) < targetThresh && abs(tag.rawPose.z-targetZ) < targetThresh && abs(pitch-pTarget) < pitchThresh)) {
                        //o.motorStop();



                        if ((abs(tag.rawPose.x-targetX) < targetThresh && abs(tag.rawPose.z-targetZ) < targetThresh && abs(tag.rawPose.z-targetZ) < targetThresh && abs(pitch) < pitchThresh)) {
                            tele.addData("xtag", tag.ftcPose.x);
                            tele.addData("ztag", tag.ftcPose.y);
                            tele.addData("yawtag", yaw);
                            tele.addData("rolltag", roll);
                            tele.addData("pitchtag", pitch);

                            tele.addData("idtag", tag.id);
                            tele.addData("wiggle", wiggle);
                            tele.update();
                            positioned = true;
                        }
                        if (positioned) {
                            TrajectorySequence t = o.rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0)).lineToConstantHeading(new Vector2d(xOffset, yOffset)).build();
                            o.rr.followTrajectorySequence(t);
                            positioned = false;
                        }
                        continue;
                    }

                    if (xCoefficient == 0 && zCoefficient == 0) xCoefficient = 0.00001;
                    TrajectorySequence t = o.rr.trajectorySequenceBuilder(new Pose2d(0, 0, rCoefficient * pitch)).lineToLinearHeading(new Pose2d((tag.ftcPose.y-targetZ)*zCoefficient, (tag.ftcPose.x-targetX)*xCoefficient, pTarget)).build();
                    o.rr.followTrajectorySequence(t);

                    // with this math, itll approach from an angle because x depends partly on z,
                    // because the farther away the tag is, the closer to the middle it will be because camera perspective
                    // (the angle will match the fov degrees of the camera)
                    tele.addData("xtag", tag.ftcPose.x);
                    tele.addData("ztag", tag.ftcPose.y);
                    tele.addData("idtag", tag.id);

                    tele.addData("yawtag", yaw);
                    tele.addData("rolltag", roll);
                    tele.addData("pitchtag", pitch);
                    tele.addData("wiggle", wiggle);
                    tele.update();
                }
            }
        }
    }
}
