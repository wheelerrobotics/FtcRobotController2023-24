package com.example.meepmeeptesting;

public class mmt {
/*
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(800);
        int prop = 1;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeBlueLight())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -65, Math.toRadians(90.00)))
                                //.splineTo(new Vector2d(-47.00, -40.67), Math.toRadians(120.00)) //#1 spline
                                //.splineTo(new Vector2d(-36.44, -36.27), Math.toRadians(89.02)) //#2 spline
                                .splineTo(new Vector2d(5.46, -40.31), Math.toRadians(140.00))
                                .splineTo(new Vector2d(12, -44), Math.toRadians(90.00))
                                .splineTo(new Vector2d(16.46, -45.31), Math.toRadians(70))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(24, -60, Math.toRadians(0)), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(38, -40.79), Math.toRadians(0.00))

                                .build()
                );

        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }*/
}
