package org.firstinspires.ftc.teamcode.autotests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.AprilDet;
import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.robot.boats.Odo;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp
@Config
public class TagFollowerOpMode extends LinearOpMode {
    public static double targetX = -1;
    public static double targetZ = 2;
    public static double targetID = 1 ;
    public static double zCoefficient = -1;
    public static double xCoefficient = -1;
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
        AprilDet ad = new AprilDet();
        ad.init(hardwareMap, "Webcam 1");


        waitForStart();

        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> a = ad.getDetected();
            AprilTagDetection tag = null;
            if (a != null) {
                if (a.size() > 0) {
                    for (AprilTagDetection d : a) {
                        if (d.id == targetID) {
                            tag = d;
                            break;
                        }
                    }
                    if (tag == null) {
                        o.motorStop();
                        continue;
                    }
                    pidx.setTarget(targetX);
                    pidz.setTarget(targetZ);
                    double pidxTick = pidx.tick(tag.pose.x);
                    double pidzTick = pidx.tick(tag.pose.z);
                    // with this math, itll approach from an angle because x depends partly on z,
                    // because the farther away the tag is, the closer to the middle it will be because camera perspective
                    // (the angle will match the fov degrees of the camera)
                    o.motorDriveXYVectors(xCoefficient * pidxTick, zCoefficient * pidzTick, 0);

                    tele.addData("tick x", pidxTick);
                    tele.addData("tick z", pidzTick);
                    tele.addData("x", a.get(0).pose.x);
                    tele.addData("z", a.get(0).pose.z);
                    tele.addData("id", a.get(0).id);
                    tele.update();
                }
            }
        }
    }
}
