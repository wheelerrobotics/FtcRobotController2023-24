package org.firstinspires.ftc.teamcode.demos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.BotVision;
import org.firstinspires.ftc.teamcode.vision.pipelines.PropDetector;

@TeleOp
public class WebcamDemo extends LinearOpMode {
    private BotVision bv = new BotVision();
    @Override
    public void runOpMode() throws InterruptedException {
        // simple class for viewing camera feed and testing processors
        bv.init(hardwareMap, new PropDetector(true));
        waitForStart();
        while (opModeIsActive()){

        }
    }
}
