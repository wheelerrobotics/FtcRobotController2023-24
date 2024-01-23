package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.CrazyTrajectoryGenerator.genCrazyTrajectory;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double speed = 5;
        double botWidth = 16;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(16, 16)
                //.setConstraints(57*speed, 25*speed, Math.toRadians(140)*speed, Math.toRadians(60)*speed, 12.8)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeRedDark())
                .setStartPose(new Pose2d(-60, -60, 0))
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(54, 30, 0))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(54, 30, 0), new Pose2d(56,64, 0), new Pose2d(-20, 0, 0), new Pose2d( 20,0, 0), new Pose2d(-200,1,0), new Pose2d(1,1, 0)))
                                //POS 3
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(12, 64, -PI/2), new Pose2d(12,32, -PI/2), new Pose2d(0, -2, 0), new Pose2d( 0,-5, 0), new Pose2d(1,1,0), new Pose2d(1,1, 0)))
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(12,32, -PI/2), new Pose2d(36, 36, 0), new Pose2d(0, 13, 0), new Pose2d(12, -12, 0), new Pose2d(0,200,0), new Pose2d(57,0, 0)))

                                // POS 1
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(12, 64, -PI/2), new Pose2d(16,36, -PI/6), new Pose2d(1, -2, 0), new Pose2d(8,-5, 0), new Pose2d(-1,-1,0), new Pose2d(-1,-1, 0)))
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(16,36, -PI/6), new Pose2d(36, 36, 0), new Pose2d(-13, 12, -0.06), new Pose2d(0, -1, 0.06), new Pose2d(0,1000,0), new Pose2d(57,0, 0.003)))
                                // POS 3
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(12, 64, -PI/2), new Pose2d(8,36, 7*PI/6), new Pose2d(-1, -2, 0.01), new Pose2d(-8,-5, -.06), new Pose2d(-1,-1,0), new Pose2d(-1,-1, 0)))
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(8,36, 7*PI/6), new Pose2d(36, 36, 0), new Pose2d(13, 12, 0.1), new Pose2d(0, -1, 0.06), new Pose2d(0,1000,0), new Pose2d(57,0, 0.003)))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}