package org.firstinspires.ftc.teamcode.helpers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathSegment;
import com.acmerobotics.roadrunner.path.QuinticSpline;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import java.util.Arrays;

public class CrazyTrajectoryGenerator {
    static TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(DriveConstants.MAX_VEL),
            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL)
    ));
    static TrajectoryAccelerationConstraint accelConstraint = new MinAccelerationConstraint(Arrays.asList(
            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL),
            new ProfileAccelerationConstraint(DriveConstants.MAX_ANG_ACCEL)
    ));
    static Trajectory genCrazyTrajectory(Pose2d startPose, Pose2d endPose, Pose2d startD, Pose2d endD, Pose2d startD2, Pose2d endD2){
        return TrajectoryGenerator.INSTANCE.generateTrajectory(new Path(
                new PathSegment(
                        new QuinticSpline(
                            new QuinticSpline.Knot(startPose.vec(), startD.vec(), startD2.vec()),
                            new QuinticSpline.Knot(endPose.vec(), endD.vec(), endD2.vec()),
                            1, 10, 10), // no clue whst these numbers do,
                        new SplineInterpolator(
                            startPose.getHeading(),
                            endPose.getHeading(),
                            startD.getHeading(),
                            startD2.getHeading(),
                            endD.getHeading(),
                            endD2.getHeading())
                    )
        ), velConstraint, accelConstraint);
    }
    static Trajectory genCrazyTrajectory(Pose2d startPose, Pose2d endPose, Pose2d startD, Pose2d endD){
        return TrajectoryGenerator.INSTANCE.generateTrajectory(new Path(
                new PathSegment(
                        new QuinticSpline(
                                new QuinticSpline.Knot(startPose.vec(), startD.vec()),
                                new QuinticSpline.Knot(endPose.vec(), endD.vec()),
                                1, 10, 10), // no clue whst these numbers do,
                        new SplineInterpolator(
                                startPose.getHeading(),
                                endPose.getHeading(),
                                startD.getHeading(),
                                0d,
                                endD.getHeading(),
                                0d
                        )
                )
        ), velConstraint, accelConstraint);
    }
}
