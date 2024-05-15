package org.firstinspires.ftc.teamcode.demos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.AprilDet;
import org.firstinspires.ftc.teamcode.helpers.apriltag.global_position;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Config
public class AprilTagLocalizer {
    public static double offsetX = 0;
    public static double offsetY = 0;
    public static double offsetR = 0;
    global_position gp;
    public AprilDet ad = null;
    public Double prevRx = null;

    public void init(HardwareMap hardwareMap) {

        ad = new AprilDet();
        ad.init(hardwareMap, "Webcam 1");
        ElapsedTime et = new ElapsedTime();
        et.reset();
        while (et.milliseconds() < 500);
    }

    public Pose2d tryLocalize() {
        ArrayList<AprilTagDetection> dets = ad.getDetected();
        if (dets == null) return null;
        if (dets.size() == 0) return null;
        return ad.pos;

    }
}
