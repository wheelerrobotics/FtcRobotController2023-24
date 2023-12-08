package org.firstinspires.ftc.teamcode.autotests;

import static java.lang.Math.PI;

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

@TeleOp
@Config
public class PushbotPixelPlacerRB extends LinearOpModeDebug {
    public static double handBasePos = 0.58;
    public static double handPushPos = 0.82;
    public static double mx = 0.5;
    public static double bx = 0;
    public static double my = 0.5;
    public static double by = 0;

    public static double x1 = 24;
    public static double y1 = 12;
    public static double r1 = 0.5;


    public static double x2 = 30;
    public static double y2 = 0;
    public static double r2 = 0;


    public static double x3 = 28;
    public static double y3 = -12;
    public static double r3 = -0.5;

    public static double tagx = 24;
    public static double tagy = 44;
    public static double tagr = PI/2;

    public static double secondx = 18;
    public static double secondy = 12;
    public static double secondr = PI/2;

    public static int exp = 10;


    private BotVision bv = new BotVision();
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            Odo o = new Odo();
            o.init(hardwareMap);
            o.autoinit();
            //      ___
            //     /   \   berd
            //     \˚∆˚/
            //      / \
            //     /| |\
            //    /-M M-\
            FtcDashboard dash = FtcDashboard.getInstance();
            Telemetry tele = dash.getTelemetry();

            bv.init(hardwareMap, new PropDetector(false));
            sleep(2500);
            //bv.webcam.getExposureControl().setExposure(10, TimeUnit.MILLISECONDS);

            o.setHandPos(handBasePos);
            while (opModeInInit()) {
                bv.webcam.getExposureControl().setMode(ExposureControl.Mode.ContinuousAuto);
                bv.webcam.getExposureControl().setMode(ExposureControl.Mode.AperturePriority);
                if (isStopRequested()) break;

                //bv.webcam.getExposureControl().setExposure(exp, TimeUnit.MILLISECONDS);
            }
            waitForStart();
            o.setHandPos(handBasePos);

            // while (opModeIsActive()){
            int pos = bv.getPos();
            if (pos == 0) pos = 2;
            bv.webcam.closeCameraDevice();

            tele.addData("pos", pos);
            tele.update();
            TrajectorySequence traj = null;
            TrajectorySequence traj2 = null;
            o.rr.setPoseEstimate(new Pose2d(0,0,0));

            if (pos == 1) {
                traj = o.rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0)).lineToLinearHeading(new Pose2d(x1, y1, r1)).lineToLinearHeading(new Pose2d(secondx, secondy, secondr)).build();
                traj2 = o.rr.trajectorySequenceBuilder(traj.end()).lineToLinearHeading(new Pose2d(tagx, tagy-0.5, tagr)).build();
            }
            else if (pos == 2) {
                traj = o.rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0)).lineToLinearHeading(new Pose2d(x2, y2, r2)).lineToLinearHeading(new Pose2d(secondx, secondy, secondr)).build();
                traj2 = o.rr.trajectorySequenceBuilder(traj.end()).lineToLinearHeading(new Pose2d(31, tagy, tagr)).build();
            }
            else if (pos == 3) {
                traj = o.rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0)).lineToLinearHeading(new Pose2d(24, 0, r3)).lineToLinearHeading(new Pose2d(x3, y3, r3)).lineToLinearHeading(new Pose2d(secondx, secondy, secondr)).build();
                traj2 = o.rr.trajectorySequenceBuilder(traj.end()).lineToLinearHeading(new Pose2d(40, tagy-0.5, tagr)).build();
            }

            AprilDet ad = new AprilDet();
            ad.init(hardwareMap, "Webcam 1");

            tele.addData("pos", pos);
            tele.update();
            if (traj != null) o.rr.followTrajectorySequence(traj);
            //relocalize(o, ad);
            sleep(100);
            if (traj2 != null) o.rr.followTrajectorySequence(traj2);
            
            o.setHandPos(handPushPos);
            sleep(3000);
            o.setHandPos(handBasePos);
            o.rr.followTrajectorySequence(o.rr.trajectorySequenceBuilder(traj2.end()).lineTo(new Vector2d(3, 35)).lineTo(new Vector2d(0, 52)).build());
            sleep(3000);
            o.rr.followTrajectorySequence(o.rr.trajectorySequenceBuilder(o.rr.getPoseEstimate()).lineToLinearHeading(new Pose2d(0, 0, 0)).build());



            
        } catch (Exception e) {
            FtcDashboard.getInstance().getTelemetry().addData("err", e.getMessage());
            FtcDashboard.getInstance().getTelemetry().update();
        }
    }
    public void relocalize(Odo o, AprilDet ad) {
        double x = ad.getDetected().get(0).center.x;
        double y = ad.getDetected().get(0).center.y;
        o.rr.setPoseEstimate(new Pose2d(x * mx + bx, y * my + by, o.rr.getPoseEstimate().getHeading()));
    }
}
