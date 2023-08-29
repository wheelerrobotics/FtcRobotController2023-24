package org.firstinspires.ftc.teamcode.demos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class DcMotorControllerDemo extends LinearOpMode {
    public double power = 0;
    public DcMotorEx motor = null;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "testMotor");
        waitForStart();
        while(opModeIsActive()) {
            power = gamepad1.left_stick_y;
            motor.setPower(power);
        };
    }
}
