package org.firstinspires.ftc.teamcode.helpers.apriltag;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.openftc.apriltag.AprilTagDetection;

public class Globalpositioning {

    public static AngleUnit outputUnitsAngle = AngleUnit.RADIANS;

    public Globalpositioning() { // learn this syntax

    }

    public static Pose2d globalPositionToPose(global_position position) {
        return new Pose2d(position.global_x, position.global_y, position.rotation_z);
    }

    public static AprilTagPoseFtc findPose(AprilTagDetection detection) {
        AprilTagPoseFtc ftcPose = new AprilTagPoseFtc();

        ftcPose.x = detection.pose.x;
        ftcPose.y = detection.pose.z;
        ftcPose.z = -detection.pose.y;

        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ,
                outputUnitsAngle);
        ftcPose.yaw = -rot.firstAngle;
        ftcPose.roll = rot.thirdAngle;
        ftcPose.pitch = rot.secondAngle;

        ftcPose.range = Math.hypot(ftcPose.x, ftcPose.y);
        ftcPose.bearing = outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(-ftcPose.x, ftcPose.y));
        ftcPose.elevation = outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(ftcPose.z, ftcPose.y));

        return ftcPose;
    }

    public static global_position find_global_pose(AprilTagDetection detection) // get id from findPose static,
    {

        AprilTagPoseFtc ftcPose = findPose(detection);
        double mounted_z_cam = .3; // include more here
        global_position bot_position = new global_position();
        global_position tag_detected = new global_position();
        double detected_id = detection.id;

        if (detected_id == tag_positions.left_tag_blue.id) {
            tag_detected = tag_positions.left_tag_blue;
        }
        if (detected_id == tag_positions.right_tag_blue.id) {
            tag_detected = tag_positions.right_tag_blue;
        }
        if (detected_id == tag_positions.left_tag_red.id) {
            tag_detected = tag_positions.left_tag_red;
        }
        if (detected_id == tag_positions.right_tag_red.id) {
            tag_detected = tag_positions.right_tag_red;
        }
        if (detected_id == tag_positions.center_tag_red.id) {
            tag_detected = tag_positions.center_tag_red;
        }
        if (detected_id == tag_positions.center_tag_blue.id) {
            tag_detected = tag_positions.center_tag_blue;
        }
        if (detected_id == tag_positions.wall_tag_left.id) {
            tag_detected = tag_positions.wall_tag_left;
        }
        if (detected_id == tag_positions.wall_tag_right.id) {
            tag_detected = tag_positions.wall_tag_right;
        }
        bot_position.range = ftcPose.range;
        bot_position.global_y = (tag_detected.global_y - ((ftcPose.range * 1/0.0254) * Math.cos(ftcPose.elevation) * Math.cos(ftcPose.yaw - ftcPose.bearing)));
        bot_position.global_x = (tag_detected.global_x - (ftcPose.range * 1/0.0254) * Math.cos(ftcPose.elevation) * Math.sin(ftcPose.yaw - ftcPose.bearing));

        bot_position.global_z = (mounted_z_cam - (tag_detected.global_z + (ftcPose.range * 1/0.0254) * Math.sin(ftcPose.elevation))); // is this right?

        bot_position.rotation_z = (tag_detected.rotation_z + ftcPose.yaw);
        bot_position.rotation_y = (tag_detected.rotation_y + ftcPose.roll);
        bot_position.rotation_x = (tag_detected.rotation_x + ftcPose.pitch);

        bot_position.id = 252;

        return bot_position;
    }

}
