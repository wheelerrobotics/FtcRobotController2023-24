package org.firstinspires.ftc.teamcode.autotests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.AprilDet;
import org.firstinspires.ftc.teamcode.robot.boats.Odo;
import org.firstinspires.ftc.teamcode.vision.BotVision;

@TeleOp
@Config
@Disabled
public class Clays extends LinearOpModeDebug {
    public  double handBasePos = 0.58;
    public  double handPushPos = 0.77;
    public static double mx = 0.5;
    public static double bx = 0;
    public static double my = 0.5;
    public static double by = 0;


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

            AprilDet ad = new AprilDet();
            ad.init(hardwareMap, "Webcam 1");
            waitForStart();
            while (opModeIsActive()) {
                relocalize(o, ad);
                double x = ad.getDetected().get(0).center.x * mx + bx;
                double y = ad.getDetected().get(0).center.y * my + by;



                tele.addData("xc", x);
                tele.addData("yc", y);
                tele.addData("x", o.rr.getPoseEstimate().getX());
                tele.addData("y", o.rr.getPoseEstimate().getY());
                tele.addData("r", o.rr.getPoseEstimate().getHeading());
                tele.update();
            }




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
