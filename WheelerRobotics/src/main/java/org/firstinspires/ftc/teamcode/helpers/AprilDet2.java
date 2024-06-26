package org.firstinspires.ftc.teamcode.helpers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;
@Config
public class AprilDet2 {
    public static int exposure = 0;
    public static int gain = 0;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    public VisionPortal visionPortal;
    private Telemetry telemetry;

    public void init(HardwareMap hardwareMap) {
        telemetry = FtcDashboard.getInstance().getTelemetry();
        initAprilTag(hardwareMap);

        // Save more CPU resources when camera is no longer needed.

    }   // end method runOpMode()
    public void stop() {
        visionPortal.close();
    }
    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag(HardwareMap hardwareMap) {

        // Create the AprilTag processor the easy way.

        aprilTag = new AprilTagProcessor.Builder().setLensIntrinsics(1044.825321498012, 1044.6104225946867, 633.7313077534989, 329.2186566305057)
                .build();
        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
        visionPortal.resumeLiveView();

    }   // end method initAprilTag()

    /**
     * Function to add telemetry about AprilTag detections.
     */
    public List<AprilTagDetection> currentDetections;
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }
    public List<AprilTagDetection> getDetectionsTele() {
        telemetryAprilTag();
        return currentDetections;
    }
    private void telemetryAprilTag() {
        setManualExposure(exposure, gain);

        currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                FtcDashboard.getInstance().getTelemetry().addLine(String.format("XYR %6.1f %6.1f %6.1f  (inch, inch, deg)", detection.ftcPose.range*Math.sin(detection.ftcPose.yaw - detection.ftcPose.bearing), detection.ftcPose.range*Math.cos(detection.ftcPose.yaw - detection.ftcPose.bearing), detection.ftcPose.yaw));
                FtcDashboard.getInstance().getTelemetry().update();
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    private boolean    setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                //sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        //if (!isStopRequested())
        //{
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                //sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            //sleep(20);

            // Set Gain.
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            //sleep(20);
            return (true);
        //} else {
        //    return (false);
        //}
    }
}
