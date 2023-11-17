package org.firstinspires.ftc.teamcode.demos;

import static org.firstinspires.ftc.teamcode.helpers.AprilDet.exposureMillis;
import static org.firstinspires.ftc.teamcode.helpers.AprilDet.gainMillis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.BotVision;
import org.firstinspires.ftc.teamcode.vision.pipelines.GlobalPositionPipeline;

import java.util.concurrent.TimeUnit;

@TeleOp
@Config
public class WebcamDemo extends LinearOpMode {
    public static double d = 0;
    public static double r = 3;
    public static double x = 1;
    public static double y = 3.2216;
    public static double xp = 0;
    public static double yp = -131.3;
    private BotVision bv = new BotVision();
    @Override
    public void runOpMode() throws InterruptedException {
        // simple class for viewing camera feed and testing processors
            GlobalPositionPipeline gpp = new GlobalPositionPipeline(0.166, 1044.825321498012, 1044.6104225946867, 633.7313077534989, 329.2186566305057);
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
    }
}
