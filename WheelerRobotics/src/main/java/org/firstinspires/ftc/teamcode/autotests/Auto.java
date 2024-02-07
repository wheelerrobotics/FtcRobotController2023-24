package org.firstinspires.ftc.teamcode.autotests;

import static org.firstinspires.ftc.teamcode.robot.boats.Bert.tiltPlacePos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.PropAprilDet;
import org.firstinspires.ftc.teamcode.robot.boats.Bert;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

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
    public static TrajectorySequence placerBlueAnyHeight(Bert b, double prop, MarkerCallback incrementer, double height) {
        return b.rr.trajectorySequenceBuilder(new Pose2d(36, 44, 0))
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(52, prop == 1 ? 42 : (prop == 2 ? 36 : 28), 0))
                .waitSeconds(0.8)
                .addTemporalMarker(0, () -> {
                    b.setSlideTarget(height);
                })
                .addTemporalMarker(0.2, () -> {
                    b.setClawOpen(false);
                })
                .addTemporalMarker(0.6, () -> {
                    b.setTilt(tiltPlacePos + 0.06);
                    b.setArmPickup(false);
                })
                .addTemporalMarker(1.8, () -> {
                    b.setClawOpen(true);
                })
                .addTemporalMarker(1.9, () -> {
                    b.setSlideTarget(height + 600);
                })
                .addTemporalMarker(2.2, incrementer)
                .build();
    }
    public static TrajectorySequence placerBlueLow(Bert b, double prop, MarkerCallback incrementer) {
        return b.rr.trajectorySequenceBuilder(new Pose2d(36, 44, 0))
                .waitSeconds(0.3)
                .lineToLinearHeading(new Pose2d(52, prop == 1 ? 44 : (prop == 2 ? 36 : 28), 0))
                .waitSeconds(0.8)
                .addTemporalMarker(0, () -> {
                    b.setSlideTarget(500);
                })
                .addTemporalMarker(0.2, () -> {
                    b.setClawOpen(false);
                })
                .addTemporalMarker(0.6, () -> {
                    b.setTilt(tiltPlacePos + 0.06);
                    b.setArmPickup(false);
                })
                .addTemporalMarker(1.8, () -> {
                    b.setClawOpen(true);
                })
                .addTemporalMarker(1.9, () -> {
                    b.setSlideTarget(1000);
                })
                .addTemporalMarker(2.2, incrementer)
                .build();
    }
}
