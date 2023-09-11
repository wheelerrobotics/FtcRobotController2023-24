package org.firstinspires.ftc.teamcode.demos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.vision.BotVision;
import org.firstinspires.ftc.teamcode.vision.pipelines.ConePipeline;

@TeleOp
public class WebcamDemo extends LinearOpMode {
    private BotVision bv = new BotVision();
    @Override
    public void runOpMode() throws InterruptedException {
        // simple class for viewing camera feed and testing processors
        DcMotor motor = hardwareMap.get(DcMotor.class, "testMotor");
        bv.init(hardwareMap, new ConePipeline());
        waitForStart();
        while (opModeIsActive()){

        }
    }
}
