package org.firstinspires.ftc.teamcode.demos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class DcMotorConfigDemo extends LinearOpMode {
    public static double power = 0;
    public DcMotorEx motor = null;
    public FtcDashboard dash;
    public Telemetry tele;
    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();
        motor = hardwareMap.get(DcMotorEx.class, "testMotor");
        waitForStart();
        while(opModeIsActive()) {
            motor.setPower(power);
        };
    }
}
