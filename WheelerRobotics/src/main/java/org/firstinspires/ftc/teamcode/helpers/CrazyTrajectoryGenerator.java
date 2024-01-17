package org.firstinspires.ftc.teamcode.helpers;

import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.ACCEL_CONSTRAINT;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.VEL_CONSTRAINT;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathSegment;
import com.acmerobotics.roadrunner.path.QuinticSpline;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.acmerobotics.roadrunner.trajectory.constraints.MinAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import java.util.Arrays;

@Config
public class CrazyTrajectoryGenerator {
    static double maxDK = 100;
    static double maxSegLen = 100;
    static int maxDepth = 100;

    static TrajectoryAccelerationConstraint accelConstraint = new MinAccelerationConstraint(Arrays.asList(
            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL),
            new ProfileAccelerationConstraint(DriveConstants.MAX_ANG_ACCEL)
    ));
    public static Trajectory genCrazyTrajectory(Pose2d startPose, Pose2d endPose, Pose2d startD, Pose2d endD, Pose2d startD2, Pose2d endD2){
        return TrajectoryGenerator.INSTANCE.generateTrajectory(new Path(
                new PathSegment(
                        new QuinticSpline(
                            new QuinticSpline.Knot(startPose.vec(), startD.vec(), startD2.vec()),
                            new QuinticSpline.Knot(endPose.vec(), endD.vec(), endD2.vec()),
                            maxDK, maxSegLen, maxDepth), // no clue whst these numbers do,
                        new SplineInterpolator(
                            startPose.getHeading(),
                            endPose.getHeading(),
                            startD.getHeading(),
                            startD2.getHeading(),
                            endD.getHeading(),
                            endD2.getHeading())
                    )
        ), VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }
    static Trajectory genCrazyTrajectory(Pose2d startPose, Pose2d endPose, Pose2d startD, Pose2d endD){
        return TrajectoryGenerator.INSTANCE.generateTrajectory(new Path(
                new PathSegment(
                        new QuinticSpline(
                                new QuinticSpline.Knot(startPose.vec(), startD.vec()),
                                new QuinticSpline.Knot(endPose.vec(), endD.vec()),
                                maxDK, maxSegLen, maxDepth), // no clue whst these numbers do,
                        new SplineInterpolator(
                                startPose.getHeading(),
                                endPose.getHeading(),
                                startD.getHeading(),
                                0d,
                                endD.getHeading(),
                                0d
                        )
                )
        ), VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }
}
