package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.CrazyTrajectoryGenerator.genCrazyTrajectory;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting4 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double speed = 5;
        double botWidth = 16;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(16, 16)
                //.setConstraints(57*speed, 25*speed, Math.toRadians(140)*speed, Math.toRadians(60)*speed, 12.8)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeRedDark())

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 64, 0))
                                //POS 3
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(-36, 64, -PI/2), new Pose2d(-28,36, -PI/6), new Pose2d(1, -2, 0), new Pose2d(8,-5, PI/30), new Pose2d(1,-1,0), new Pose2d(1,-1, 0)))
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(-28,36, -PI/6), new Pose2d(40, 24, 0), new Pose2d(-120, 70, 0), new Pose2d(12, 12, 0), new Pose2d(0,-1800,0), new Pose2d(57,0, 0)))
                                // POS 2
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(-36, 64, -PI/2), new Pose2d(-36, 32, -PI/2), new Pose2d(0, -12, 0), new Pose2d(0,0, 0), new Pose2d(0,0,0), new Pose2d(0,0, 0)))
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(-36,32, -PI/2), new Pose2d(40, 36, 0), new Pose2d(0, 150, 0.05), new Pose2d(12, 90, -0.02), new Pose2d(-2000,-2200,0), new Pose2d(57,0, 0)))
                                // POS 1
                                .addTrajectory(genCrazyTrajectory(new Pose2d(-36, 64, -PI/2), new Pose2d(-40,36, -5*PI/6), new Pose2d(-1, -2, 0), new Pose2d(-8,-5, 0), new Pose2d(1,-1,0), new Pose2d(1,-1, 0)))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(-40,36, -5*PI/6), new Pose2d(40, 24, 0), new Pose2d(130, 120, 0.06), new Pose2d(12, 24, -0.06), new Pose2d(-2050,-2400,0), new Pose2d(57,0, -0.003)))

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}