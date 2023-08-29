package org.firstinspires.ftc.teamcode.demos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PID;

@Config
@TeleOp
public class MotorPidDemo extends LinearOpMode {
    public static double target = 0;
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public DcMotorEx motor = null;
    public FtcDashboard dash;
    public Telemetry tele;
    public PID pid;
    public double power, position;
    @Override
    public void runOpMode() throws InterruptedException {

        dash = FtcDashboard.getInstance();
        tele = dash.getTelemetry();
        motor = hardwareMap.get(DcMotorEx.class, "testMotor");
        pid = new PID(kp, ki, kd, false);

        waitForStart();
        while (opModeIsActive()) {
            position = motor.getCurrentPosition();
            tele.addData("target", target);
            tele.addData("position", position);
            tele.update();
            pid.setConsts(kp, ki, kd);
            pid.setTarget(target);

            power = pid.tick(position);

            motor.setPower(power);
        }

    }
}
