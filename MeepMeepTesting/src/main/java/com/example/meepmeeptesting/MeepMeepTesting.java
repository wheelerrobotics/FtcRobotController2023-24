package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.CrazyTrajectoryGenerator.genCrazyTrajectory;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathSegment;
import com.acmerobotics.roadrunner.path.QuinticSpline;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Arrays;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double speed = 5;
        double botWidth = 16;
        QuinticSpline pos3Spline = new QuinticSpline(
                new QuinticSpline.Knot(new Vector2d(-36, -72+botWidth/2), new Vector2d(1, 2), new Vector2d(0,0)),
                new QuinticSpline.Knot(new Vector2d(-28,-36), new Vector2d(8,5), new Vector2d(0,0)),
        100, 100, 100);
        SplineInterpolator pos3SplineInter = new SplineInterpolator(PI/2, PI/6, -PI/60, 0d, -PI/60, 0d);
        PathSegment ps = new PathSegment(pos3Spline, pos3SplineInter);
        Path p = new Path(ps);
        TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(57*speed),
                new AngularVelocityConstraint(Math.toRadians(140)*speed)
        ));
        TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(25*speed);
        Trajectory traj = TrajectoryGenerator.INSTANCE.generateTrajectory(p, velConstraint, accelConstraint);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(16, 16)
                //.setConstraints(57*speed, 25*speed, Math.toRadians(140)*speed, Math.toRadians(60)*speed, 12.8)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeRedDark())
                .setStartPose(new Pose2d(-60, -60, 0))
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -72+botWidth/2, 0))
                                //POS 3
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(-36, -72+botWidth/2, PI/2), new Pose2d(-28,-36, PI/6), new Pose2d(1, 2, 0), new Pose2d(8,5, -PI/30), new Pose2d(1,1,0), new Pose2d(1,1, 0)))
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(-28,-36, PI/6), new Pose2d(40, -24, 0), new Pose2d(-120, -70, 0), new Pose2d(12, -12, 0), new Pose2d(0,1800,0), new Pose2d(57,0, 0)))
                                // POS 2
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(-36, -72+botWidth/2, PI/2), new Pose2d(-36,-24-botWidth/2, PI/2), new Pose2d(0, 12, 0), new Pose2d(0,0, 0), new Pose2d(0,0,0), new Pose2d(0,0, 0)))
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(-36,-24-botWidth/2, PI/2), new Pose2d(40, -36, 0), new Pose2d(0, -150, -0.05), new Pose2d(12, -90, 0.02), new Pose2d(-2000,2200,0), new Pose2d(57,0, 0)))
                                // POS 1
                                .addTrajectory(genCrazyTrajectory(new Pose2d(-36, -72+botWidth/2, PI/2), new Pose2d(-40,-36, 5*PI/6), new Pose2d(-1, 2, 0), new Pose2d(-8,5, 0), new Pose2d(1,1,0), new Pose2d(1,1, 0)))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(-40,-36, 5*PI/6), new Pose2d(40, -24, 0), new Pose2d(130, -120, -0.06), new Pose2d(12, -24, 0.06), new Pose2d(-2050,2400,0), new Pose2d(57,0, 0.003)))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}