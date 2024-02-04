package org.firstinspires.ftc.teamcode.autotests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.PropAprilDet;
import org.firstinspires.ftc.teamcode.robot.boats.Bert;

public class Auto {
    public static int getPropPos(PropAprilDet ad, int timeout, int defaultPos) {
        ElapsedTime cooldown = new ElapsedTime();
        ad.setWeBeProppin(true);
        cooldown.reset();
        while (cooldown.milliseconds() < timeout) {
            ad.tick();
        }
        ad.tick();
        return ad.getProp() == 0 ? defaultPos : ad.getProp();
    }
    public static int getPropPos(PropAprilDet ad, int timeout) {
        ElapsedTime cooldown = new ElapsedTime();
        ad.setWeBeProppin(true);
        cooldown.reset();
        while (cooldown.milliseconds() < timeout) {
            ad.tick();
        }
        ad.tick();
        return ad.getProp() == 0 ? 2 : ad.getProp();
    }
    public static void relocalize(Bert b, PropAprilDet ad, Pose2d target, int count) {
        int localizationCount = 0;
        while (localizationCount < count) {
            ad.setWeBeProppin(false);
            ad.getDetected(); // update

            if (ad.pos != null)
                b.rr.setPoseEstimate(new Pose2d(ad.pos.getX() - 7.875, ad.pos.getY() + 7, b.rr.getPoseEstimate().getHeading()));
            b.rr.followTrajectorySequence(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate()).lineToLinearHeading(target).build());
            //curMoveID++;
            localizationCount++;
        }
        return;
    }
    public static Trajectory incrementer(Bert b, MarkerCallback callback) {
        return b.rr.trajectoryBuilder(new Pose2d(0,0)).lineTo(new Vector2d(0.01, 0)).addTemporalMarker(0, callback).build();
    }
}
