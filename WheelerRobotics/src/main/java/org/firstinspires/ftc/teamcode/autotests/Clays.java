package org.firstinspires.ftc.teamcode.autotests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.boats.Odo;

@Autonomous
@Config
public class Clays extends LinearOpModeDebug {

    public static double handBasePos = 0.58;
    public static int millis = 5000;
    public static double handPushPos = 0.77;
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            Odo o = new Odo();

            o.init(hardwareMap);
            o.autoinit();
            //    ___
            //   /   \   berd
            //   \˚∆˚/
            //    / \
            //   /| |\
            //  /-M M-\
            FtcDashboard dash = FtcDashboard.getInstance();
            Telemetry tele = dash.getTelemetry();

            waitForStart();

            while (opModeIsActive()) {
                o.setHandPos(handBasePos);

                sleep(millis);
                o.setHandPos(handPushPos);
                sleep(millis);
            }

        } catch (Exception e) {
                FtcDashboard.getInstance().getTelemetry().addData("ERR", e.getMessage());
                FtcDashboard.getInstance().getTelemetry().update();
        }
    }
}
