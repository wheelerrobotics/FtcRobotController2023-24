package org.firstinspires.ftc.teamcode.autotests.main;

import static org.firstinspires.ftc.teamcode.autotests.Auto.incrementer;
import static org.firstinspires.ftc.teamcode.autotests.Auto.placerBlueAnyHeight;
import static org.firstinspires.ftc.teamcode.autotests.Auto.placerBlueLow;
import static org.firstinspires.ftc.teamcode.helpers.CrazyTrajectoryGenerator.genCrazyTrajectory;
import static org.firstinspires.ftc.teamcode.helpers.CrazyTrajectoryGenerator.genCrazyTrajectoryConstrained;
import static org.firstinspires.ftc.teamcode.robot.boats.Bert.leftShuvDown;
import static org.firstinspires.ftc.teamcode.robot.boats.Bert.leftShuvUp;
import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.PropAprilDet;
import org.firstinspires.ftc.teamcode.robot.boats.Bert;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

//GOOD
@Autonomous
public class BlueBackdropPlus extends LinearOpMode {
    public int localizationCount = 0;
    ElapsedTime cooldown;
    int curMoveID = 0;
    boolean firstTimeSlides = true;
    double prop = 0;
    Bert b = null;
    PropAprilDet ad = null;
    Telemetry tele = null;
    boolean done = true;
    TrajectorySequence kuwait = null;
    Thread newThread = null;
    ExecutorService threadpool = Executors.newCachedThreadPool();
    Future<TrajectorySequence> futureTask = null;

    @Override
    public void runOpMode() throws InterruptedException {
        double c = 0;
        double botWidth = 16;
        b = new Bert();
        b.setCawtFailsafe(false);
        b.init(hardwareMap);
        ad = new PropAprilDet();
        ad.init(hardwareMap, "Front", false);
        cooldown = new ElapsedTime();
        tele = FtcDashboard.getInstance().getTelemetry();
        while (opModeInInit()) {

            b.rr.setPoseEstimate(new Pose2d(12, 64, -PI/2));
            if (ad.bv.opened) {
                ad.setWeBeProppin(true);
                ad.tick();
            }
        }

        while (opModeIsActive()) {
            if (futureTask != null) {
                if (futureTask.isDone()) {
                    try {
                        kuwait = futureTask.get();
                    } catch (ExecutionException e) {
                        throw new RuntimeException(e);
                    }
                    futureTask.cancel(true);
                    threadpool.shutdown();
                }
            }
            if (curMoveID ==0) {
                if (done) {
                    done = false;
                    ad.setWeBeProppin(true);
                    cooldown.reset();
                    while (cooldown.milliseconds() < 500) {
                        ad.tick();
                    }
                    ad.tick();
                    prop = ad.getProp();
                    if (prop != 0) { // add a timeout or make getprop rly robust
                        setKuwait(prop);
                        if (prop == 1) {
                            b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())
                                    .addTrajectory(genCrazyTrajectoryConstrained(new Pose2d(12, 64, -PI / 2), new Pose2d(17, 39, -PI / 6), new Pose2d(1, -2, 0), new Pose2d(8, -5, 0), new Pose2d(-1, -1, 0), new Pose2d(-1, -1, 0), 30, 20))
                                    .addTrajectory(genCrazyTrajectory(new Pose2d(15, 36, -PI / 6), new Pose2d(36, 44, 0), new Pose2d(-13, 12, -0.06), new Pose2d(0, -1, 0.06), new Pose2d(0, 1000, 0), new Pose2d(57, 0, 0.003)))
                                    .addTrajectory(incrementer(b, increment))

                                    .build());
                        } else if (prop == 2) {
                            b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())

                                    .addTrajectory(genCrazyTrajectoryConstrained(new Pose2d(12, 64, -PI / 2), new Pose2d(12, 38, -PI / 2), new Pose2d(0, -2, 0), new Pose2d(0, -2, 0), new Pose2d(1, 1, 0), new Pose2d(1, 1, 0), 30, 20))
                                    .addTrajectory(genCrazyTrajectory(new Pose2d(12, 35, -PI / 2), new Pose2d(36, 44, 0), new Pose2d(0, 13, 0), new Pose2d(12, -12, 0), new Pose2d(0, 200, 0), new Pose2d(57, 0, 0)))
                                    .addTrajectory(incrementer(b, increment))

                                    .build());
                        } else {
                            b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate())
                                    .addTrajectory(genCrazyTrajectoryConstrained(new Pose2d(12, 64, -PI / 2), new Pose2d(11, 36, 7 * PI / 6), new Pose2d(-1, -2, 0.01), new Pose2d(-8, -5, -.06), new Pose2d(-1, -1, 0), new Pose2d(-1, -1, 0), 30, 20))
                                    .addTrajectory(genCrazyTrajectory(new Pose2d(8, 36, 7 * PI / 6), new Pose2d(36, 44, 0), new Pose2d(13, 12, 0.1), new Pose2d(0, -1, 0.06), new Pose2d(0, 1000, 0), new Pose2d(57, 0, 0.003)))
                                    .addTrajectory(incrementer(b, increment))

                                    .build());
                        }

                    }
                }

            }

            if (curMoveID == 1) {
                relocalize(new Pose2d(36, 44, 0), 2);
                done = true;
            }
            if (curMoveID == 2) {
                if (done){
                    done = false;
                    b.rr.followTrajectorySequenceAsync(placerBlueLow(b, prop, increment));
                }
            }
            if (curMoveID == 3) {
                if (done){
                    done = false;
                    b.rr.followTrajectorySequenceAsync(kuwait);
                }
            }
            if (curMoveID == 4) {
                relocalize(new Pose2d(36, 44, 0), 1);
                done = true;
            }
            if (curMoveID == 5) {
                if (done){
                    done = false;
                    b.rr.followTrajectorySequenceAsync(placerBlueAnyHeight(b, 2, increment, 1000));
                }
            }
            if (curMoveID == 6) {
                if (done){
                    done = false;
                    Pose2d curpos = new Pose2d(57, 36, 0);//prop == 1 ? new Pose2d(54, 44, 0) : (prop == 2 ? new Pose2d(54, 36, 0) : new Pose2d(54, 28, 0));
                    b.rr.followTrajectorySequenceAsync(b.rr.trajectorySequenceBuilder(curpos)
                            .back(10)
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
                            .addTemporalMarker(2, increment)

                            .build());
                }
            }
            if (curMoveID == 7) {
                threadpool.shutdown();
                return;
            }
            b.autoTick();
            tele.addData("curmove", curMoveID);
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
                tele.addData("tagy", ad.pos.getY() + 6);
                tele.update();
            }
            if (ad.pos != null)
                b.rr.setPoseEstimate(new Pose2d(ad.pos.getX() - 7.875, ad.pos.getY() + 7, b.rr.getPoseEstimate().getHeading()));
            b.rr.followTrajectorySequence(b.rr.trajectorySequenceBuilder(b.rr.getPoseEstimate()).lineToLinearHeading(target).build());
            localizationCount++;
        }
        localizationCount = 0;
        return;
    }
    public void setKuwait(double pos) {
        futureTask = threadpool.submit(() -> {
            Pose2d curpos = pos == 1 ? new Pose2d(54, 44, 0) : (pos == 2 ? new Pose2d(54, 36, 0) : new Pose2d(54, 28, 0));
            return b.rr.trajectorySequenceBuilder(curpos)
                    .addTrajectory(genCrazyTrajectory(curpos, new Pose2d(-40, 10, 0), new Pose2d(-100, -12, 0), new Pose2d(40, 110, 0), new Pose2d(1000,-1200 + (pos - 1) * 300,0), new Pose2d(1000,pos == 1 ? 1300 : 1400, 0)))
                    .addTrajectory(genCrazyTrajectory(new Pose2d(-60,  0, 0), new Pose2d(-59, 0, 0), new Pose2d(-2, 0, 0), new Pose2d(-1, 0, 0), new Pose2d(0,0,0), new Pose2d(0,0, 0)))

                    .addTrajectory(genCrazyTrajectory(new Pose2d(-59, 14, 0), new Pose2d(-60, 16, 0), new Pose2d(0, 2, 0), new Pose2d(0, -2, 0.1), new Pose2d(-4,0,0), new Pose2d(0,0, 0)))
                    .waitSeconds(0.7)
                    .addTrajectory(genCrazyTrajectory(new Pose2d(-61, 22, 0), new Pose2d(36, 44, 0), new Pose2d(20, -20, 0), new Pose2d(0, 100, 0), new Pose2d(100,-200,0), new Pose2d(300,-100, 0)))
                    .addTemporalMarker(1, () -> {
                        b.setClawOpen(true);
                        b.setArmPickup(true);
                        b.setTiltPickup(true);
                        b.setSlideTarget(-80);
                        b.setDownCorrectionFactor(0.2);
                        b.setDownCorrection(true);
                        b.spintake(-0.8);
                        b.setLeftShuv(leftShuvDown);
                    })
                    // shuv down
                    .addTemporalMarker(6.8, ()->{
                        b.spintake(1);
                    })
                    .addTemporalMarker(8, ()->{
                        b.spintake(0);
                    })
                    .addTemporalMarker(8.5, ()->{
                        b.spintake(-1);
                    })

                    .addTemporalMarker(9, ()->{
                        b.spintake(0);
                    })
                    //.addSpatialMarker(new Vector2d(40, 20), ()->{
                    //b.spintake(1);
                    //}) // intake on
                    //.addSpatialMarker(new Vector2d(-30, 20), ()->{
                    //b.spintake(0);
                    //}) // intake reverse
                    /*.addSpatialMarker(new Vector2d(-30, 20), ()->{
                        b.setDownCorrection(false);
                    }) // intake off & shuv up
*/
                    .addTrajectory(incrementer(b, increment))
                    .build();
        });

    }
    public MarkerCallback increment = () ->{ done = true; };
}
