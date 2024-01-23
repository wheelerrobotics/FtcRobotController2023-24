package org.firstinspires.ftc.teamcode.helpers;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.helpers.apriltag.Globalpositioning;
import org.firstinspires.ftc.teamcode.helpers.apriltag.global_position;
import org.firstinspires.ftc.teamcode.vision.BotVision;
import org.firstinspires.ftc.teamcode.vision.pipelines.DualAprilPropPipeline;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Config
public class PropAprilDet {
    public global_position gp = null;
    public Pose2d pos = null;
    Double prevRx = null;
    public static int exposureMillis = 500;
    public static int gainMillis = 1;

    public ArrayList<AprilTagDetection> detections = new ArrayList<>();
    public BotVision bv = new BotVision();
    public DualAprilPropPipeline dapp = null;
    // these are the intrinsics for the microsoft lifecam HD 3000, running at 640x480 resolution
    int curConePos = 0;
    int numFramesWithoutDetection = 0;
    public static float DECIMATION_LOW = -1;
    public static float DECIMATION_HIGH = -1;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 2.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 7;

    public double xOffset = 7;
    public double yOffset = 7;

    public boolean nb = false;
    public double rOverride = 0;

    public void init(HardwareMap hw, String webcamName, boolean notBlue) {
        nb = notBlue;
        dapp = new DualAprilPropPipeline(notBlue, 0.05, 672.384, 672.384, 322.894, 253.854);
        bv.init(hw, dapp, webcamName);
        ElapsedTime et = new ElapsedTime();
        et.reset();
        //while (et.milliseconds() < 500); // THIS MIGHT BREAK IF IT GETS COMMENTED BUT TRY ANYWAY




    }
    public void setWeBeProppin(boolean proppin) {
        dapp.weBeProppin = proppin;
        tick();
    }
    public int getProp() {
        return (dapp.weBeProppin) ? dapp.pos : 0;
    }
    public void tick() {
        if(dapp.weBeProppin) {
            bv.webcam.getExposureControl().setMode(ExposureControl.Mode.AperturePriority);
        }else{
            bv.webcam.getExposureControl().setMode(ExposureControl.Mode.Manual);
            bv.webcam.getExposureControl().setExposure(exposureMillis, TimeUnit.MICROSECONDS);
            //bv.webcam.getExposureControl().setMode(ExposureControl.Mode.Manual);
            //bv.webcam.getExposureControl().setExposure(exposureMillis, TimeUnit.MICROSECONDS);
            //bv.webcam.getGainControl().setGain(gainMillis);
        }
    }
    public ArrayList<AprilTagDetection> getDetected(){
        return checkDetections();
    }
    public ArrayList<AprilTagDetection> checkDetections() {
        tick();
        if (dapp.weBeProppin) {
            return null;
        }

        //bv.webcam.getExposureControl().setMode(ExposureControl.Mode.Manual);
        //bv.webcam.getExposureControl().setExposure(exposureMillis, TimeUnit.MICROSECONDS);
        //bv.webcam.getGainControl().setGain(gainMillis);


        FtcDashboard.getInstance().getTelemetry().addData("DEGUG", "updates");
        FtcDashboard.getInstance().getTelemetry().update();
        detections = dapp.getDetectionsUpdate();

        FtcDashboard.getInstance().getTelemetry().addData("DEGUG", "bef null");
        FtcDashboard.getInstance().getTelemetry().update();
        if (detections != null) {

            // If we don't see any tags
            if (detections.size() == 0) {

                FtcDashboard.getInstance().getTelemetry().addData("DEGUG", "zero size");
                FtcDashboard.getInstance().getTelemetry().update();
                numFramesWithoutDetection++;

                // If we haven't seen a tag for a few frames, lower the decimation
                // so we can hopefully pick one up if we're e.g. far back
                if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                    dapp.setDecimation(DECIMATION_LOW);
                }
            }
            // We do see tags!
            else {
                numFramesWithoutDetection = 0;

                // If the target is within 1 meter, turn on high decimation to
                // increase the frame rate
                if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                    dapp.setDecimation(DECIMATION_HIGH);
                }

                FtcDashboard.getInstance().getTelemetry().addData("DEGUG", "deitz");
                FtcDashboard.getInstance().getTelemetry().update();
                if (detections.size() == 0) return detections;
                FtcDashboard.getInstance().getTelemetry().addData("DEGUG", "dets nonnull");
                FtcDashboard.getInstance().getTelemetry().update();
                double totalX = 0;
                double totalY = 0;
                double totalR = 0;
                double totalRx = 0;

                // filter flips

                for (AprilTagDetection i : detections) {
                    if (i.id != (nb ? 5 : 2)) continue;
                    gp = Globalpositioning.find_global_pose(i);
                    if (gp.rotation_x > 0.3) continue;

                    totalX += gp.global_x;
                    totalY += gp.global_y;
                    totalR += gp.rotation_z;
//GOOD
                    totalRx += gp.rotation_x;
                }

                double avgRx = totalRx;//detections.size();
                if (prevRx == null) {
                    prevRx = avgRx;
                    return detections;
                }
                if (Math.abs(avgRx) > .1+Math.abs(prevRx) && prevRx != null) return detections;
                prevRx = avgRx;

                FtcDashboard.getInstance().getTelemetry().addData("DEGUG", "totaled");
                FtcDashboard.getInstance().getTelemetry().update();
                double weightedX = totalX;//detections.size();
                double weightedY = totalY;//detections.size();
                double weightedR = totalR;//detections.size();
                FtcDashboard.getInstance().getTelemetry().addData("DEGUG", "averaged " + weightedX + " " + weightedY + " " + weightedR + " ");
                FtcDashboard.getInstance().getTelemetry().update();
                if (weightedX == 0 && weightedY == 0 && weightedR == 0) return detections;
                if (rOverride != 0) pos = new Pose2d((weightedY-sin(-rOverride)), -(weightedX*cos(-rOverride)), -weightedR);
                else pos = new Pose2d((weightedY-sin(-weightedR)), -(weightedX*cos(-weightedR)), -weightedR);
            }

        }





        return detections;
    }

}
