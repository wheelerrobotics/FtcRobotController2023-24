package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.CrazyTrajectoryGenerator.genCrazyTrajectory;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MMT {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double speed = 5;
        double botWidth = 16;
        int prop = 1;
        Pose2d curpos = prop == 1 ? new Pose2d(54, 44, 0) : (prop == 2 ? new Pose2d(54, 36, 0) : new Pose2d(54, 28, 0));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(16, 16)
                //.setConstraints(57*speed, 25*speed, Math.toRadians(140)*speed, Math.toRadians(60)*speed, 12.8)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -64, -PI/2))
                                //BS1
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(-36, 64, -PI/2), new Pose2d(-32,36, -PI/6), new Pose2d(1, -2, 0), new Pose2d(8,-5, 0), new Pose2d(-1,-1,0), new Pose2d(-1,-1, 0)))
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(-33, 36, -PI/6), new Pose2d(36, 40, 0), new Pose2d(-80, 40, 0.03), new Pose2d(0, 90, 0), new Pose2d(-400, -1800, -0.001), new Pose2d(1500, 500, 0)))
                                // BS2
                                //.lineToLinearHeading(new Pose2d(-36,34, -PI/2))
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(-36, 65, -PI / 2), new Pose2d(-36, 32, -PI / 2), new Pose2d(0.1, 0, 0), new Pose2d(-0.1, 0, 0), new Pose2d(0, 0, 0), new Pose2d(0, 0, 0)))
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(-36, 32, -PI / 2), new Pose2d(36, 40, 0), new Pose2d(0, 150, 0.05), new Pose2d(12, 190, -0.02), new Pose2d(-2300, -2800, 0), new Pose2d(1200, 1500, 0)))
                                //BS3
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(-36, 64, -PI/2), new Pose2d(-39,36, -5*PI/6), new Pose2d(-1, -2, 0.0), new Pose2d(-8,-5, 0.06), new Pose2d(-1,-1,0), new Pose2d(-1,-1, 0)))

                                //.addTrajectory(genCrazyTrajectory(new Pose2d(-36, 64, -PI / 2), new Pose2d(-37, 36,  -5*PI / 6), new Pose2d(-1, -2, 0), new Pose2d(-8, -5, 0), new Pose2d(1, -1, 0), new Pose2d(1, -1, 0)))
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(-39, 36,  -5*PI / 6), new Pose2d(36, 40, 0), new Pose2d(130, 100, 0.05), new Pose2d(0, 24, -0.06), new Pose2d(-2350, -2500, 0), new Pose2d(800, -100, -0.003)))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(12, -64, PI / 2), new Pose2d(17, -39, PI / 6), new Pose2d(1, 2, 0), new Pose2d(8, 5, 0), new Pose2d(-1, 1, 0), new Pose2d(-1, 1, 0)))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(12, -64, PI / 2), new Pose2d(12, -38, PI / 2), new Pose2d(0, 2, 0), new Pose2d(0, 2, 0), new Pose2d(1, -1, 0), new Pose2d(1, -1, 0)))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(12, -64, PI / 2), new Pose2d(11, -36, -7 * PI / 6), new Pose2d(-1, 2, -0.01), new Pose2d(-8, 5, .06), new Pose2d(-1, 1, 0), new Pose2d(-1, 1, 0)))


                                //RS 3
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(-36, -64, PI/2), new Pose2d(-38,-36, 5*PI/6), new Pose2d(-1, 2, 0.0), new Pose2d(-8,5, 0), new Pose2d(-1,1,0), new Pose2d(-1,1, 0)))
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(-37, 36,  -5*PI / 6), new Pose2d(36, 40, 0), new Pose2d(130, 100, 0.06), new Pose2d(0, 24, -0.06), new Pose2d(-2350, -2400, 0), new Pose2d(1007, -100, -0.003)))
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(-33, -36, PI / 6), new Pose2d(-36, -50, 0), new Pose2d(0,0,0), new Pose2d(0,0,0), new Pose2d(0,0,0), new Pose2d(0,0,0)))
                                //.lineToLinearHeading(new Pose2d(-36, -54, 0))
                                //.addTrajectory(genCrazyTrajectory(new Pose2d(-36, -54, 0), new Pose2d(36, -40, 0), new Pose2d(0, -30, 0), new Pose2d(0, 30, 0), new Pose2d(0, 0, 0), new Pose2d(500, -300, 0)))


                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}