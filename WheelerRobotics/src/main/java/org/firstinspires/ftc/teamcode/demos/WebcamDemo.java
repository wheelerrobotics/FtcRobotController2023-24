package org.firstinspires.ftc.teamcode.demos;

import static org.firstinspires.ftc.teamcode.helpers.RelativePoseFinder.findPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.AprilDet;
import org.firstinspires.ftc.teamcode.helpers.apriltag.Globalpositioning;
import org.firstinspires.ftc.teamcode.robot.boats.Odo;
import org.firstinspires.ftc.teamcode.vision.BotVision;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

import java.util.concurrent.TimeUnit;

@TeleOp
public class WebcamDemo extends LinearOpMode {
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
            o.motorDriveXYVectors(gamepad1.);
            ArrayList<AprilTagDetection> dets = ad.getDetected();
            if (dets == null) continue;
            for (AprilTagDetection i : dets) {
                global_position gp = Globalpositioning.find_gglobal_pose(i);

            }

        }
        while (opModeIsActive()){
            try{
                ArrayList<AprilTagDetection> dets = ad.getDetected();
                if (dets == null) continue;
                for (AprilTagDetection i : dets) {
                    if (i.id == 5) {
                        tele.addData("center x", i.center.x);
                        tele.addData("center y", i.center.y);
                        tele.addData("id", i.id);
                        tele.addData("decisionMargin", i.decisionMargin);
                        tele.addData("hamming", i.hamming);
                        tele.addData("pose z", i.pose.z);
                        tele.addData("pose x", i.pose.x);
                        tele.addData("pose y", i.pose.y);
                        Orientation orientation = Orientation.getOrientation(i.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.RADIANS);
                        tele.addData("pose firstAngle", orientation.firstAngle); // rl
                        tele.addData("pose secondAngle", orientation.secondAngle); // updown
                        tele.addData("pose thirdAngle", orientation.thirdAngle); // roll



    }
}
