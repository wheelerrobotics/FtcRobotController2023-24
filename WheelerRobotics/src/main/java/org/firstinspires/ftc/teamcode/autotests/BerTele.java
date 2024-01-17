package org.firstinspires.ftc.teamcode.autotests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.PropAprilDet;
import org.firstinspires.ftc.teamcode.robot.boats.Bert;

@Config
@TeleOp
public class BerTele extends LinearOpMode {
    public static double target = 0;


    /*

    Controller 1:




     */
    @Override
    public void runOpMode() throws InterruptedException {
        Bert b = new Bert();
        b.init(hardwareMap);
        PropAprilDet ad = new PropAprilDet();
        ad.init(hardwareMap, "Webcam 2");

        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        while (opModeInInit()) {

        }
        while (opModeIsActive()) {

            b.motorDriveXYVectors(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            if (gamepad2.right_bumper && gamepad2.left_bumper) {
                b.driveLeftHang(gamepad2.left_stick_y);
                b.driveRightHang(gamepad2.right_stick_y);
            }else {
                b.spintake(gamepad2.right_stick_y);
            }
            if (!(gamepad2.right_bumper && gamepad2.left_bumper)){
                b.driveHangs(gamepad2.right_trigger);
            }
            if (gamepad1.right_trigger != 0 || gamepad1.left_trigger != 0) b.driveSlides(gamepad1.right_trigger - gamepad1.left_trigger);
            if (gamepad1.a) {
                b.setSlideTarget(target);
            }
            tele.addData("lt", gamepad1.left_trigger);
            tele.addData("rt", gamepad1.right_trigger);
            tele.update();
            b.tick();
        }
    }
}
