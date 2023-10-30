package org.firstinspires.ftc.teamcode.autotests;

import static org.firstinspires.ftc.teamcode.helpers.RelativePoseFinder.findPose;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.AprilDet;
import org.firstinspires.ftc.teamcode.robot.boats.Odo;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.openftc.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp
@Config
public class TagFollowerOpModeRRCus extends LinearOpMode {
    public static double targetThresh = 0.05;
    public static double aaronsVariable = 0.1;
    public static double targetX = -1.97;
    public static double targetZ = 4.1;
    public static double targetID = 1;
    public static double zCoefficient = 0.68;
    public static double xCoefficient = -1.1;
    public static double rCoefficient = 0.065;
    public static double xOffset = 6;
    public static double yOffset = -1;
    public static double wiggleMag = 0.4;
    public static double wiggleThresh = 40;
    public static double pitchThresh = 0.05;
    public static double pTarget = 0;
    public volatile static boolean positioned = false;
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
        AprilDet ad = new AprilDet();
        ad.init(hardwareMap, "Webcam 1");


        waitForStart();
        boolean positioned = false;
        int wiggle = 0;

        while (opModeIsActive()) {
            List<AprilTagDetection> a = ad.getDetected();
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

                    AprilTagPoseFtc pose = findPose(tag);
                    //MatrixF rot = tag.pose.R;

                    //double yaw = atan2(rot.get(1, 0),rot.get(0,0));
                    //double roll = atan2(rot.get(2, 1),rot.get(2,2));
                    //double pitch = -asin(rot.get(2, 0)); // THIS ONE
                    // this is so exciting


                    double head = o.rr.getRawExternalHeading();

                    if (tag == null || (abs(pose.x-targetX) < targetThresh && abs(pose.z-targetZ) < targetThresh)) {
                        //o.motorStop();



                        if ((abs(pose.x-targetX) < targetThresh && abs(pose.z-targetZ) < targetThresh)) {
                            tele.addData("xtag", pose.x);
                            tele.addData("ztag", pose.z);
                            tele.addData("ytag", pose.y);
                            tele.addData("yawtag", pose.yaw);
                            tele.addData("rolltag", pose.roll);
                            tele.addData("pitchtag", pose.pitch);
                            tele.addData("idtag", tag.id);

                            tele.addData("wiggle", wiggle);
                            tele.addData("positioned", true);
                            tele.update();
                            positioned = true;
                        }
                        if (positioned) {
                            TrajectorySequence t = o.rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0)).lineToConstantHeading(new Vector2d(xOffset, yOffset)).build();
                            o.rr.followTrajectorySequence(t);
                            continue;
                            //positioned = false;
                        }
                    }

                    tele.addData("xtag", pose.x);
                    tele.addData("ztag", pose.z);
                    tele.addData("ytag", pose.y);
                    tele.addData("yawtag", pose.yaw);
                    tele.addData("rolltag", pose.roll);
                    tele.addData("pitchtag", pose.pitch);
                    tele.addData("idtag", tag.id);
                    tele.addData("positioned", false);
                    tele.update();


                    if (xCoefficient == 0 && zCoefficient == 0) xCoefficient = 0.00001;
                    TrajectorySequence t = o.rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0)).lineToLinearHeading(new Pose2d((pose.z-targetZ)*zCoefficient, (pose.x-targetX)*xCoefficient, (head-pTarget) * rCoefficient)).build();
                    o.rr.followTrajectorySequence(t);
                    /*
*/
                    // with this math, itll approach from an angle because x depends partly on z,
                    // because the farther away the tag is, the closer to the middle it will be because camera perspective
                    // (the angle will match the fov degrees of the camera)

                }
            }
        }
    }
}
