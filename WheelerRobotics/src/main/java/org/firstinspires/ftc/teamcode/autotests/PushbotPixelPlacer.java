package org.firstinspires.ftc.teamcode.autotests;

import static org.firstinspires.ftc.teamcode.helpers.RelativePoseFinder.findPose;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.helpers.AprilDet;
import org.firstinspires.ftc.teamcode.robot.boats.Odo;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BotVision;
import org.firstinspires.ftc.teamcode.vision.pipelines.PropDetector;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.openftc.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp
@Config
public class PushbotPixelPlacer extends LinearOpModeDebug {

    public static double x1 = 7;
    public static double y1 = 4;
    public static double r1 = -0.04;


    public static double x2 = 8;
    public static double y2 = 3;
    public static double r2 = 0;


    public static double x3 = 7;
    public static double y3 = -2.5;
    public static double r3 = 0.04;

    public static int exp = 10;


    private BotVision bv = new BotVision();
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            Odo o = new Odo();
            o.init(hardwareMap);
            o.autoinit();
            //    ___
            //   /   \   berd
            //   \˚∆˚/
            //    / \
            //   /| |\
            //  /-M M-\
            FtcDashboard dash = FtcDashboard.getInstance();
            Telemetry tele = dash.getTelemetry();

            bv.init(hardwareMap, new PropDetector(false));
            sleep(2500);
            //bv.webcam.getExposureControl().setExposure(10, TimeUnit.MILLISECONDS);

            while (opModeInInit()) {
                bv.webcam.getExposureControl().setMode(ExposureControl.Mode.ContinuousAuto);
                bv.webcam.getExposureControl().setMode(ExposureControl.Mode.AperturePriority);


                //bv.webcam.getExposureControl().setExposure(exp, TimeUnit.MILLISECONDS);
            }
            waitForStart();

            // while (opModeIsActive()){
            int pos = bv.getPos();

            tele.addData("pos", pos);
            tele.update();
            TrajectorySequence traj = null;
            if (pos == 1)
                traj = o.rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0)).lineToLinearHeading(new Pose2d(x1, y1, r1)).lineTo(new Vector2d(3, 4)).lineToLinearHeading(new Pose2d(7, 7, -0.05)).build();
            else if (pos == 2)
                traj = o.rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0)).lineToLinearHeading(new Pose2d(x2, y2, r2)).lineTo(new Vector2d(7, 4)).lineToLinearHeading(new Pose2d(7, 6, -0.1)).build();
            else if (pos == 3)
                traj = o.rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0)).lineToLinearHeading(new Pose2d(x3, y3, r3)).lineTo(new Vector2d(7, 4)).lineToLinearHeading(new Pose2d(7, 6, -0.1)).build();

            tele.addData("pos", pos);
            tele.update();
            if (traj != null) o.rr.followTrajectorySequence(traj);

            bv.webcam.closeCameraDevice();
            AprilDet ad = new AprilDet();
            ad.init(hardwareMap, "Webcam 1");

            while (opModeIsActive()) {
                while (o.rr.isBusy()) ;
                boolean positioned = false;
                double targetThresh = 0.05;
                double targetX = -1.97;
                double targetZ = 4.1;
                double zCoefficient = 0.68;
                double targetID = 1;
                double xCoefficient = -1.1;
                double rCoefficient = 0.065;
                double xOffset = 6;
                double yOffset = -1;
                double wiggleMag = 0.4;
                double wiggleThresh = 40;
                double pitchThresh = 0.05;
                double pTarget = 0;
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

                            if (tag == null) {
                                tele.addData("wiggle", true);
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

                            if (tag == null || (abs(pose.x - targetX) < targetThresh && abs(pose.z - targetZ) < targetThresh)) {
                                //o.motorStop();


                                if ((abs(pose.x - targetX) < targetThresh && abs(pose.z - targetZ) < targetThresh)) {
                                    tele.addData("xtag", pose.x);
                                    tele.addData("ztag", pose.z);
                                    tele.addData("ytag", pose.y);
                                    tele.addData("yawtag", pose.yaw);
                                    tele.addData("rolltag", pose.roll);
                                    tele.addData("pitchtag", pose.pitch);
                                    tele.addData("idtag", tag.id);

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
                            TrajectorySequence t = o.rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0)).lineToLinearHeading(new Pose2d((pose.z - targetZ) * zCoefficient, (pose.x - targetX) * xCoefficient, (head - pTarget) * rCoefficient)).build();
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
        } catch (Exception e) {
            FtcDashboard.getInstance().getTelemetry().addData("err", e.getMessage());
            FtcDashboard.getInstance().getTelemetry().update();
        }
    }
}
