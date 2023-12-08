package org.firstinspires.ftc.teamcode.demos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.AprilDet;
import org.firstinspires.ftc.teamcode.helpers.apriltag.Globalpositioning;
import org.firstinspires.ftc.teamcode.helpers.apriltag.global_position;
import org.firstinspires.ftc.teamcode.robot.boats.Odo;
import org.firstinspires.ftc.teamcode.vision.BotVision;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp
@Config
public class WebcamDemo extends LinearOpMode {
    public static double d = 0;
    public static double r = 3;
    public static double x = 1;
    public static double y = 1;//3.2216;
    public static double xp = 0;
    public static double yp = 0;//-131.3;
    private BotVision bv = new BotVision();
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        Odo o = new Odo();
        // simple class for viewing camera feed and testing processors
        AprilDet ad = new AprilDet();
        ad.init(hardwareMap, "Webcam 1");
        waitForStart();


        while (opModeIsActive()) {
            o.motorDriveXYVectors(gamepad1);
            ArrayList<AprilTagDetection> dets = ad.getDetected();
            if (dets == null) continue;
            for (AprilTagDetection i : dets) {
                global_position gp = Globalpositioning.find_global_pose(i);

            }

        }

    }
}
