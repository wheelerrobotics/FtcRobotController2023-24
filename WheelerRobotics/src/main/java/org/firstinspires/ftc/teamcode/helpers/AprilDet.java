package org.firstinspires.ftc.teamcode.helpers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vision.BotVision;
import org.firstinspires.ftc.teamcode.vision.pipelines.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Config
public class AprilDet {

    public ArrayList<AprilTagDetection> detections = new ArrayList<>();
    public BotVision bv = null;
    public AprilTagDetectionPipeline atdp =  new AprilTagDetectionPipeline(0.166, 578.272, 578.272, 402.145, 221.506);
    int curConePos = 0;
    int numFramesWithoutDetection = 0;
    public static int DECIMATION_LOW = 5;
    public static int DECIMATION_HIGH = 5;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 2.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 7;

    public void init(HardwareMap hw, String webcamName) {
        bv = new BotVision();
        ElapsedTime et = new ElapsedTime();
        et.reset();
        while (et.milliseconds() < 500);
        bv.init(hw, atdp, webcamName);


    }
    public ArrayList<AprilTagDetection> getDetected(){
        return checkDetections();
    }
    public ArrayList<AprilTagDetection> checkDetections() {
        detections = atdp.getDetectionsUpdate();
        if (detections != null) {

            // If we don't see any tags
            if (detections.size() == 0) {
                numFramesWithoutDetection++;

                // If we haven't seen a tag for a few frames, lower the decimation
                // so we can hopefully pick one up if we're e.g. far back
                if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                    atdp.setDecimation(DECIMATION_LOW);
                }
            }
            // We do see tags!
            else {
                numFramesWithoutDetection = 0;

                // If the target is within 1 meter, turn on high decimation to
                // increase the frame rate
                if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                    atdp.setDecimation(DECIMATION_HIGH);
                }
            }

        }
        return detections;
    }

}
