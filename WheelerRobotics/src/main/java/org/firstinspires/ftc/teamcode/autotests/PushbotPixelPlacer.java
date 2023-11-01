package org.firstinspires.ftc.teamcode.autotests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.robot.boats.Odo;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BotVision;
import org.firstinspires.ftc.teamcode.vision.pipelines.PropDetector;

@TeleOp
@Config
public class PushbotPixelPlacer extends LinearOpModeDebug {

    public static double x1 = 5;
    public static double y1 = -2;
    public static double r1 = -0.2;


    public static double x2 = 7;
    public static double y2 = 7;
    public static double r2 = 0;


    public static double x3 = 5;
    public static double y3 = 2;
    public static double r3 = 0.2;

    public static int exp = 10;


    private BotVision bv = new BotVision();
    @Override
    public void runOpMode() throws InterruptedException {
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

            //bv.webcam.getExposureControl().setExposure(exp, TimeUnit.MILLISECONDS);
        }
        waitForStart();

       // while (opModeIsActive()){
        int pos = bv.getPos();

        tele.addData("pos", pos);
        tele.update();
        TrajectorySequence traj = null;
        if (pos == 1) traj = o.rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0)).lineToLinearHeading(new Pose2d(x1, y1, r1)).build();
        else if (pos == 2) traj = o.rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0)).lineToLinearHeading(new Pose2d(x2, y2, r2)).build();
        else if (pos == 3) traj = o.rr.trajectorySequenceBuilder(new Pose2d(0, 0, 0)).lineToLinearHeading(new Pose2d(x3, y3, r3)).build();

        tele.addData("pos", pos);
        tele.update();
        if (traj != null) o.rr.followTrajectorySequence(traj);


        while (opModeIsActive()) {
        }
    }
}
