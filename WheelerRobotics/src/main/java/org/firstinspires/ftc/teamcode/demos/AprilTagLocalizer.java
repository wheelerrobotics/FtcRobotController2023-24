package org.firstinspires.ftc.teamcode.demos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.AprilDet;
import org.firstinspires.ftc.teamcode.helpers.apriltag.Globalpositioning;
import org.firstinspires.ftc.teamcode.helpers.apriltag.global_position;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Config
public class AprilTagLocalizer {
    public static double offsetX = 0;
    public static double offsetY = 0;
    public static double offsetR = 0;
    global_position gp;
    AprilDet ad = new AprilDet();

    public void init(HardwareMap hardwareMap) {

        AprilDet ad = new AprilDet();
        ad.init(hardwareMap, "Webcam 1");
    }

    public Pose2d tryLocalize() {
        ArrayList<AprilTagDetection> dets = ad.getDetected();
        if (dets == null) return null;
        double totalX = 0;
        double totalY = 0;
        double totalR = 0;
        for (AprilTagDetection i : dets) {
            gp = Globalpositioning.find_global_pose(i);

            totalX += gp.global_x;
            totalY += gp.global_y;
            totalR += gp.rotation_z;
        }
        double weightedX = totalX/dets.size();
        double weightedY = totalY/dets.size();
        double weightedR = totalR/dets.size();

        return new Pose2d(weightedX-offsetX, weightedY-offsetY, weightedR-offsetR);
    }
}
