package org.firstinspires.ftc.teamcode.demos;

import static org.firstinspires.ftc.teamcode.helpers.AprilDet.exposureMillis;
import static org.firstinspires.ftc.teamcode.helpers.AprilDet.gainMillis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.helpers.AprilDet;
import org.firstinspires.ftc.teamcode.vision.BotVision;
import org.firstinspires.ftc.teamcode.vision.pipelines.GlobalPositionPipeline;

import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
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
        // simple class for viewing camera feed and testing processors
            GlobalPositionPipeline gpp = new GlobalPositionPipeline(0.050, 1044.825321498012, 1044.6104225946867, 633.7313077534989, 329.2186566305057);
            bv.init(hardwareMap, gpp);
            double lastx=0;
            double lasty=0;
            waitForStart();
            while (opModeIsActive()) {

                bv.webcam.getExposureControl().setExposure(exposureMillis, TimeUnit.MILLISECONDS);
                bv.webcam.getGainControl().setGain(gainMillis);
                TelemetryPacket packet = new TelemetryPacket();
                gpp.setDecimation((float) d);
                if (bv.getLoc() == null) {

                    packet.fieldOverlay()
                            .setFill("blue")
                            .fillCircle(lastx, lasty, r);
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                    FtcDashboard.getInstance().getTelemetry().update();
                    continue;
                }
                packet.fieldOverlay()
                        .setFill("blue")
                        .fillCircle(x * bv.getLoc().getX() + xp, y*bv.getLoc().getY() + yp, r);
                lasty = y*bv.getLoc().getY() + yp;
                lastx = x * bv.getLoc().getX() + xp;
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                FtcDashboard.getInstance().getTelemetry().addData("X", x * bv.getLoc().getX() + xp);
                FtcDashboard.getInstance().getTelemetry().addData("Y", y*bv.getLoc().getY() + yp);
                FtcDashboard.getInstance().getTelemetry().addData("R", bv.getLoc().getHeading());
                FtcDashboard.getInstance().getTelemetry().update();
            }
            bv.webcam.closeCameraDevice();
      
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
                        tele.addData("pose firstAngle", orientation.firstAngle); // rl
                        tele.addData("pose secondAngle", orientation.secondAngle); // updown
                        tele.addData("pose thirdAngle", orientation.thirdAngle); // roll

                        AprilTagPoseFtc ftcDet = findPose(i);
                        tele.addData("supposed x", ftcDet.x);
                        tele.addData("supposed y", ftcDet.y);
                        tele.addData("supposed z", ftcDet.z);
                        tele.addData("supposed roll", ftcDet.roll);
                        tele.addData("supposed pitch", ftcDet.pitch);
                        tele.addData("supposed yaw", ftcDet.yaw);
                        tele.addData("supposed range", ftcDet.range * 1/0.0254); //53.75/0.533 far 34/0.344
                        tele.addData("supposed x offset", (ftcDet.range * 1/0.0254) * Math.cos(ftcDet.elevation) * Math.cos(ftcDet.yaw));
                        tele.addData("supposed z offset", (ftcDet.range * 1/0.0254) * Math.cos(ftcDet.elevation) * Math.sin(ftcDet.yaw));
                        tele.addData("supposed bearing", ftcDet.bearing);
                        tele.addData("supposed elevation", ftcDet.elevation);
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
