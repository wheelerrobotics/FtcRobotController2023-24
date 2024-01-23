package org.firstinspires.ftc.teamcode.autotests.main;

import static org.firstinspires.ftc.teamcode.helpers.CrazyTrajectoryGenerator.genCrazyTrajectory;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.PropAprilDet;
import org.firstinspires.ftc.teamcode.robot.boats.Bert;

@Autonomous
public class RedStacks extends LinearOpMode {
    public int localizationCount = 0;
    ElapsedTime cooldown;
    int curMoveID = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        double c = 0;
        boolean done = false;
        double botWidth = 16;
        Bert b = new Bert();
        b.init(hardwareMap);
        PropAprilDet ad = new PropAprilDet();
        ad.init(hardwareMap, "Front", true);


        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        while (opModeInInit()) {
            b.rr.setPoseEstimate(new Pose2d(-36, -64, PI/2));
            if (ad.bv.opened) {
                ad.setWeBeProppin(true);
                ad.tick();
            }
        }


        int prop = 0;
        while (opModeIsActive()) {
            ad.tick();
            if (curMoveID ==0) {
                prop = ad.getProp();
                if (prop !=0) { // add a timeout or make getprop rly robust
                    if (prop == 1) {
                        b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(new Pose2d(-36, -72 + botWidth / 2, 0))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(-36, -64, PI / 2), new Pose2d(-39, -36, 5 * PI / 6), new Pose2d(-1, 2, 0), new Pose2d(-8, 5, 0), new Pose2d(1, 1, 0), new Pose2d(1, 1, 0)))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(-39, -36, 5 * PI / 6), new Pose2d(36, -30, 0), new Pose2d(130, -100, -0.06), new Pose2d(0, -24, 0.06), new Pose2d(-2350, 2400, 0), new Pose2d(1007, -100, 0.003)))
                                .build());
                    }else if(prop == 2) {
                        b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(new Pose2d(-36, -72 + botWidth / 2, 0))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(-36, -64, PI / 2), new Pose2d(-36, -32, PI / 2), new Pose2d(0, 12, 0), new Pose2d(0, 0, 0), new Pose2d(0, 0, 0), new Pose2d(0, 0, 0)))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(-36, -32, PI / 2), new Pose2d(36, -30, 0), new Pose2d(0, -150, -0.05), new Pose2d(12, -90, 0.02), new Pose2d(-2000, 2500, 0), new Pose2d(1057, -400, 0)))
                                .build());
                    }else {
                        b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(new Pose2d(-36, -72 + botWidth / 2, 0))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(-36, -64, PI/2), new Pose2d(-34,-36, PI/6), new Pose2d(1, 2, 0), new Pose2d(8,5, 0), new Pose2d(1,1,0), new Pose2d(1,1, 0)))
                                .addTrajectory(genCrazyTrajectory(new Pose2d(-34, -36, PI / 6), new Pose2d(36, -30, 0), new Pose2d(-120, -70, 0), new Pose2d(0, -90, 0), new Pose2d(200, 1800, 0), new Pose2d(800, -500, 0)))
                                .build());
                    }
                    done = true;
                    curMoveID++;
                }
            }
            if (curMoveID == 1) {

            }

            if (curMoveID == 2) {
                done = false;
                ad.setWeBeProppin(false);
                ad.getDetected(); // update

                // offsets are for when rotation is 0
                if (ad.pos != null) {
                    tele.addData("tagx", ad.pos.getX() - 6.875);
                    tele.addData("tagy", ad.pos.getY() + 6);
                    tele.update();
                }
                if (ad.pos != null) b.rr.setPoseEstimate(new Pose2d(ad.pos.getX()-7.875, ad.pos.getY()+7, b.rr.getPoseEstimate().getHeading()));
                b.rr.followTrajectorySequence(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate()).lineToLinearHeading(new Pose2d(36, -28, 0)).build());
                //curMoveID++;
                localizationCount++;
                if (localizationCount == 2) curMoveID++;
                done = true;
            }
            if (curMoveID == 3) {
                if (!b.rr.isBusy()){
                    b.setSlideTarget(1400);
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(new Pose2d(36, -28, 0))
                            .lineToLinearHeading(new Pose2d(54, prop == 1 ? -28 : (prop == 2 ? -36 :  -44), 0))
                            .waitSeconds(3)
                            .addTemporalMarker(0, () -> {
                                b.setSlideTarget(1400);
                            })
                            .addTemporalMarker(1, () -> {
                                b.setClawOpen(false);
                            })
                            .addTemporalMarker(1.5, () -> {
                                b.setTiltPickup(false);
                                b.setArmPickup(false);
                            })
                            .addTemporalMarker(3, () -> {
                                b.setClawOpen(true);
                            })
                            .addTemporalMarker(3.2, () ->{
                                curMoveID++;
                            })
                            .build());
                }
            }
            if (curMoveID == 5) {
                if (!b.rr.isBusy()){
                    Pose2d curpos = prop == 1 ? new Pose2d(54, -28, 0) : (prop == 2 ? new Pose2d(54, -36, 0) : new Pose2d(54, -44, 0));
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(curpos)
                            .addTrajectory(genCrazyTrajectory(curpos, new Pose2d(56,-64, 0), new Pose2d(-20, 0, 0), new Pose2d( 20,0, 0), new Pose2d(-200,-1,0), new Pose2d(1,-1, 0)))
                            .addTemporalMarker(1, () -> {
                                b.setDownCorrection(true);
                                b.setDownCorrectionFactor(0.1);
                                b.setSlideTarget(0);
                            })
                            .build());
                }
            }
            b.autoTick();
            tele.addData("curmove", curMoveID);
            tele.update();
            if (!b.rr.isBusy() && done) curMoveID++;
        }
    }
}
