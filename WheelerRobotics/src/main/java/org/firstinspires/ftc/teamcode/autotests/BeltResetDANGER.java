package org.firstinspires.ftc.teamcode.autotests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class BeltResetDANGER extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor slides = hardwareMap.get(DcMotor.class, "slides");
        while (opModeInInit()) {

        }
        while (opModeIsActive()) {
            slides.setPower(0.3 * gamepad1.right_stick_y);
        }
    }
}
