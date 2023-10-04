package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.teamcode.helpers.AprilDet;
import org.firstinspires.ftc.teamcode.helpers.PID;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class TagFollower {
    PID py = null;
    PID px = null;
    Pose powers = null;
    AprilDet ad = null;
    int idx;
    public TagFollower(AprilDet aprilDet, int index) {
        ad = aprilDet;
        px = new PID(0.5, 0 , 0, false);
        py = new PID(0.5, 0 , 0, false);
        powers = new Pose(0, 0, 0);
        idx = index;
    }
    public Pose tick(double x, double y) {
        ArrayList<AprilTagDetection> det = ad.getDetected();
        AprilTagDetection tag = null;
        for (AprilTagDetection d : det) {
            if (d.id == idx) {
                tag = d;
                break;
            }
        }
        if (tag == null) return null;
        px.tick(tag.pose.x);
        py.tick(tag.pose.z);
        FtcDashboard.getInstance().getTelemetry().addData("x", tag.pose.x);
        FtcDashboard.getInstance().getTelemetry().addData("id", tag.id);
        FtcDashboard.getInstance().getTelemetry().addData("z", tag.pose.z);
        FtcDashboard.getInstance().getTelemetry().update();


        powers.x = x;
        powers.y = y;

        return powers;
    }

}
