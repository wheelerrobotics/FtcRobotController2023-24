package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.boats.Bert;

@Config
@TeleOp
public class BlinkinTests extends LinearOpMode {
    public static int number = 0;
    //public Servo s;
    public Bert b;
    @Override
    public void runOpMode() throws InterruptedException {
        b = new Bert();
        b.init(hardwareMap);
        b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //b.teleinit(hardwareMap);
        b.setDownCorrection(true);
        b.setDownCorrectionFactor(0.45);
        //PropAprilDet ad = new PropAprilDet();
        int lastNumber = 0;
        //s = hardwareMap.get(Servo.class, "blinkin");
        //ad.init(hardwareMap, "Front");

        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        //s = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        while (opModeInInit()) {
        }
        while (opModeIsActive()) {
            //b.csb.setBlinkin(RevBlinkinLedDriver.BlinkinPattern.fromNumber(number));
            double[][] list = b.csb.tick();
            tele.addData("r1", list[0][0]);
            tele.addData("g1", list[0][1]);
            tele.addData("b1", list[0][2]);
            tele.addData("a1", list[0][3]);
            tele.addData("r2", list[1][0]);
            tele.addData("g2", list[1][1]);
            tele.addData("b2", list[1][2]);
            tele.addData("a2", list[1][3]);
            tele.addData("patternNum", number);
            tele.addData("pattern", RevBlinkinLedDriver.BlinkinPattern.fromNumber(number).name());
            tele.addData("pixel1", b.getPixelStates()[0]);
            tele.addData("pixel2", b.getPixelStates()[1]);
            tele.update();
        }
    }


}
