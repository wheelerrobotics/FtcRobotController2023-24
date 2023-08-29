package org.firstinspires.ftc.teamcode.demos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ServoControllerDemo extends LinearOpMode {
    public static double position = 0;
    public Servo servo = null;
    public FtcDashboard dash;
    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();
        servo = hardwareMap.get(Servo.class, "servo");
        waitForStart();
        while(opModeIsActive()) {
            servo.setPosition(position);
        };
    }
}
