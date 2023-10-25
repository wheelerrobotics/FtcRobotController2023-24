package org.firstinspires.ftc.teamcode.autotests;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.AprilDet2;
import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.robot.boats.Odo;

import java.util.List;

@TeleOp
@Config
public class TagFollowerVan extends LinearOpMode {
    public static double targetThresh = 0.1;
    public static double targetX = -1;
    public static double targetZ = 2;
    public static double targetID = 1 ;
    public static double zCoefficient = 0;
    public static double xCoefficient = 0;
    public PID pidx, pidz, pidr;

    @Override
    public void runOpMode() throws InterruptedException {
        Odo o = new Odo();
        o.init(hardwareMap);
        o.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        o.autoinit();
        pidx = new PID(0.5, 0, 0, false);
        pidz = new PID(0.5, 0, 0, false);
        pidr = new PID(0.5, 0, 0, false);
        //    ___
        //   /   \   berd
        //   \˚∆˚/
        //    / \
        //   /| |\
        //  /-M M-\
        FtcDashboard dash = FtcDashboard.getInstance();
        Telemetry tele = dash.getTelemetry();
        AprilDet2 ad = new AprilDet2();
        ad.init(hardwareMap);


        waitForStart();

        while (opModeIsActive()) {
            List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> a = ad.getDetections();
            org.firstinspires.ftc.vision.apriltag.AprilTagDetection tag = null;
            if (a == null) {
                o.motorStop();
                tele.addData("nullarray", true);
                tele.update();
                continue;
            }
                if (a.size() > 0) {
                    for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection d : a) {
                        if (d.id == targetID) {
                            tag = d;
                            break;
                        }
                    }
                    if (tag == null || (abs(tag.rawPose.x-targetX) < targetThresh && abs(tag.rawPose.z-targetZ) < targetThresh)) {
                        o.motorDriveXYVectors(0, 0, 0);
                        tele.addData("nulltag", true);
                        tele.update();
                        continue;
                    }
                    pidx.setTarget(targetX);
                    pidz.setTarget(targetZ);
                    double pidxTick = pidx.tick(tag.rawPose.x);
                    double pidzTick = pidx.tick(tag.rawPose.z);
                    // with this math, itll approach from an angle because x depends partly on z,
                    // because the farther away the tag is, the closer to the middle it will be because camera perspective
                    // (the angle will match the fov degrees of the camera)
                    o.motorDriveXYVectors(xCoefficient * pidxTick, zCoefficient * pidzTick, 0);

                    tele.addData("tick x", pidxTick);
                    tele.addData("tick z", pidzTick);
                    tele.addData("x", tag.rawPose.x);
                    tele.addData("z", tag.rawPose.z);
                    tele.addData("id", tag.id);
                    tele.addData("nulltag", false);
                    tele.update();

            }
        }
    }
}