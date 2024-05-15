package org.firstinspires.ftc.teamcode.autotests.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.helpers.PropAprilDet;
import org.firstinspires.ftc.teamcode.robot.boats.Bert;

//GOOD THEORETCALLY
@Autonomous
@Config
public class AAARedBackr extends LinearOpMode {
    public int localizationCount = 0;
    ElapsedTime cooldown;
    int curMoveID = 0;

    public static double bx = 5.8;
    public static double by= 3.9;

    public static double mx = 1;
    public static double my= 1;
    boolean firstTimeSlides = true;
    boolean localizing = false;
    double prop = 0;
    Bert b = null;
    PropAprilDet ad = null;
    Telemetry tele = null;
    boolean done = true;
    ElapsedTime clawTimer = null;
    boolean clawTiming = false;
    boolean relocalized = false;
    @Override
    public void runOpMode() throws InterruptedException {
        double c = 0;
        double botWidth = 16;
        DriveConstants.MAX_VEL = 50;
        DriveConstants.MAX_ACCEL = 35;
        b = new Bert();
        b.setCawtFailsafe(false);
        b.init(hardwareMap);
        ad = new PropAprilDet();
        ad.init(hardwareMap, "Front", false);
        cooldown = new ElapsedTime();
        clawTimer = new ElapsedTime();
        tele = FtcDashboard.getInstance().getTelemetry();
        b.rr.setPoseEstimate(new Pose2d(-36, -65, Math.toRadians(90.00)));
        while (opModeInInit()) {

            b.rr.setPoseEstimate(new Pose2d(-36, -65, Math.toRadians(90.00)));
            if (ad.bv.opened) {
                ad.setWeBeProppin(true);
                ad.tick();
            }
        }

        while (opModeIsActive()) {
            if (curMoveID ==0) {
                if (done) {
                    done = false;

                    ad.setWeBeProppin(true);
                    cooldown.reset();
                    while (cooldown.milliseconds() < 500) {
                        ad.tick();
                        prop = ad.getProp();
                    }
                    prop = (prop == 1 ? 1 : (prop == 2) ? 3 : 2);
                }}
            localizing = true;
            /*if (curMoveID == 1) {
                relocalize(new Pose2d(36, -44, 0), 2);
                done = true;
            }

            if (curMoveID == 2) {
                if (done){
                    done = false;
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(new Pose2d(36, -44, 0))
                            .waitSeconds(0.4)
                            .lineToLinearHeading(new Pose2d(57, prop == 1 ? -46 : (prop == 2 ? -34 :  -26), 0))
                            .waitSeconds(0.8)
                            .addTemporalMarker(0, () -> {
                                b.setSlideTarget(600);
                            })
                            .addTemporalMarker(0.1, () -> {
                                b.setClawOpen(false);
                            })
                            .addTemporalMarker(0.4, () -> {
                                b.setTilt(tiltPlacePos + (prop == 1 ? 0.02 : (prop == 2 ? 0.02 : 0.02)));
                                b.setArmPickup(false);
                            })
                            .addTemporalMarker(2, () -> {
                                b.setClawOpen(true);
                            })
                            .addTemporalMarker(2.2, () -> {
                                b.setSlideTarget(1000);
                            })
                            .addTemporalMarker(2.3, increment)
                            .build());
                }
            }
            if (curMoveID == 3) {
                if (done){
                    done = false;
                    Pose2d curpos = new Pose2d(57, -36, 0);//prop == 1 ? new Pose2d(54, 44, 0) : (prop == 2 ? new Pose2d(54, 36, 0) : new Pose2d(54, 28, 0));
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(curpos)
                            .back(5)
                            .strafeRight(40)
                            .forward(10)
                            .waitSeconds(2)
                            .addTemporalMarker(0.5, () -> {
                                b.setClawOpen(true);
                                b.setArmPickup(true);
                                b.setTiltPickup(true);
                                b.setLeftShuv(leftShuvUp);
                            })
                            .addTemporalMarker(1, () -> {
                                b.setDownCorrection(true);
                                b.setDownCorrectionFactor(0.1);
                                b.setSlideTarget(0);
                            })
                            .addTemporalMarker(6, increment)

                            .build());
                }
            }
            if (curMoveID == 4) {
                return;
            }*/
            b.autoTick();
            if (localizing) {
                // still gotta figure out what tags to use
                tele.addData("localizing", true);
                ad.setWeBeProppin(false);
                ad.getDetected();
                if (ad.pos != null) {
                    tele.addData("apx", mx * ad.pos.getX() - bx);
                    tele.addData("apy", my * ad.pos.getY() - by);
                    //if (abs((ad.pos.getX() - 3.875) - b.rr.getPoseEstimate().getX()) < 5 && abs((ad.pos.getY() - 4) - b.rr.getPoseEstimate().getY()) < 5) {


                      //  b.rr.setPoseEstimate(new Pose2d(ad.pos.getX() - 3.875, ad.pos.getY() - 4, ad.pos.getHeading()));
                       // relocalized = true;
                    //}
                }
            }else {

                tele.addData("localizing", false);
            }
            if (clawTiming && clawTimer.milliseconds() > 100) {
                b.setClawOpen(!b.getClawOpen());
                clawTiming = false;
            }
            tele.addData("curmove", curMoveID);
            tele.addData("relocalized", relocalized);

            tele.addData("busy", b.rr.isBusy());
            tele.update();
            //if (!b.rr.isBusy()) done = true;

            if (done) {
                //done = false;
                curMoveID++;
                tele.addData("curmove", curMoveID);
                tele.update();
            }
        }
    }
    public void relocalize(Pose2d target, int count) {
        localizationCount = 0;
        while (localizationCount < count) {
            ad.setWeBeProppin(false);
            ad.getDetected(); // update

            // offsets are for when rotation is 0
            if (ad.pos != null) {
                tele.addData("tagx", ad.pos.getX() - 6.875);
                tele.addData("tagy", ad.pos.getY() - 6); // flipped for red :)
                tele.update();
            }
            if (ad.pos != null)
                b.rr.setPoseEstimate(new Pose2d(ad.pos.getX() - 7.875, ad.pos.getY() - 7, b.rr.getPoseEstimate().getHeading()));
            b.rr.followTrajectorySequence(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate()).lineToLinearHeading(target).build());
            localizationCount++;
        }
        localizationCount = 0;
        return;
    }
    public MarkerCallback increment = () ->{ done = true; };
}
