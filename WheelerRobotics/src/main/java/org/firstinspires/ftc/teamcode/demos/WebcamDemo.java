package org.firstinspires.ftc.teamcode.demos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.PropAprilDet;
import org.firstinspires.ftc.teamcode.helpers.apriltag.Globalpositioning;
import org.firstinspires.ftc.teamcode.helpers.apriltag.global_position;
import org.firstinspires.ftc.teamcode.robot.boats.Bert;
import org.firstinspires.ftc.teamcode.vision.BotVision;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp
@Config
public class WebcamDemo extends LinearOpMode {
    public static int tagID = 1;
    public static double d = 0;
    public static double r = 3;
    public static double x = 1;
    public static double y = 1;//3.2216;
    public static double xp = 0;
    public static double yp = 0;//-131.3;
    private BotVision bv = new BotVision();
    global_position gp;
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        Bert b = new Bert();
        b.init(hardwareMap);
        // simple class for viewing camera feed and testing processors
        PropAprilDet ad = new PropAprilDet();
        ad.init(hardwareMap, "Webcam 1");
        b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tele.addData("DEBUG", "inited");
        tele.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) ad.setWeBeProppin(true);
            if (gamepad1.b) ad.setWeBeProppin(false);
            b.motorDriveXYVectors(gamepad1.left_stick_x,-gamepad1.left_stick_y,gamepad1.right_stick_x);
            tele.addData("DEBUG", "vectors");
            tele.update();
            ArrayList<AprilTagDetection> dets = ad.getDetected();
            tele.addData("DEBUG", "detected");
            tele.update();
            if (dets == null) continue;
            double totalX = 0;
            double totalY = 0;
            double totalR = 0;
            double totalRX = 0;
            double totalRY = 0;
            double totalZ = 0;
            for (AprilTagDetection i : dets) {
                gp = Globalpositioning.find_global_pose(i);
                totalX += gp.global_x;
                totalY += gp.global_y;
                totalR += gp.rotation_z;
                totalRX += gp.rotation_x;
                totalRY += gp.rotation_y;
                totalZ += gp.global_z;
            }
            double avgX = totalX/dets.size();
            double avgY = totalY/dets.size();
            double avgR = totalR/dets.size();
            double avgRX = totalRX/dets.size();
            double avgRY = totalRY/dets.size();
            double avgZ = totalZ/dets.size();

            tele.addData("x", avgX);
            tele.addData("y", avgY);
            tele.addData("r", avgR);
            tele.addData("z", avgZ);
            tele.addData("rx", avgRX);
            tele.addData("ry", avgRY);
            tele.update();
        }

        bv.webcam.closeCameraDevice();


    }
}
