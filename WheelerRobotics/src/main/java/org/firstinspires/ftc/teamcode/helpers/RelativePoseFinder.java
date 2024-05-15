package org.firstinspires.ftc.teamcode.helpers;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.openftc.apriltag.AprilTagDetection;

public class RelativePoseFinder {
    public static AngleUnit outputUnitsAngle = AngleUnit.RADIANS;
    public static AprilTagPoseFtc findPose(AprilTagDetection detection) {
       /* AprilTagPoseFtc ftcPose = new AprilTagPoseFtc();
        ftcPose.x =  detection.pose.x;
        ftcPose.y =  detection.pose.z;
        ftcPose.z = -detection.pose.y;

        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, outputUnitsAngle);
        ftcPose.yaw = -rot.firstAngle;
        ftcPose.roll = rot.thirdAngle;
        ftcPose.pitch = rot.secondAngle;

        ftcPose.range = Math.hypot(ftcPose.x, ftcPose.y);
        ftcPose.bearing = outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(-ftcPose.x, ftcPose.y));
        ftcPose.elevation = outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(ftcPose.z, ftcPose.y));

        return ftcPose;*/
        return null;
    }
}
