package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.boats.Bert;

@Config
@TeleOp
public class servotest extends LinearOpMode {
    public static double ls = 0.5;
    public static double rs = 0.5;
    public Bert b = null;
    @Override
    public void runOpMode() throws InterruptedException {
        b = new Bert();
        b.init(hardwareMap);
        //b.teleinit(hardwareMap);

        //PropAprilDet ad = new PropAprilDet();
        //ad.init(hardwareMap, "Front");

        Telemetry tele = FtcDashboard.getInstance().getTelemetry();

        while (opModeInInit());
        while (opModeIsActive()) {
            if (gamepad1.a) {
                b.setLeftShuv(ls);
                b.setRightShuv(rs);
            }
        }
    }

}
