package org.firstinspires.ftc.teamcode.demos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.helpers.AprilDet;
import org.firstinspires.ftc.teamcode.vision.BotVision;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp
public class WebcamDemo extends LinearOpMode {
    private BotVision bv = new BotVision();
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        // simple class for viewing camera feed and testing processors
        AprilDet ad = new AprilDet();
        ad.init(hardwareMap, "Webcam 1");
        waitForStart();
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
                        tele.addData("pose firstAngle", orientation.firstAngle);
                        tele.addData("pose secondAngle", orientation.secondAngle);
                        tele.addData("pose thirdAngle", orientation.thirdAngle);
                        tele.update();
                    }
            }
            }
            catch (Exception e) {
                tele.addData("ERR!", e.getMessage());
                tele.update();
            }
        }
    }
}
