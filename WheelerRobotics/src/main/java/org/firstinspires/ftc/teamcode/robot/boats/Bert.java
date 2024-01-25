package org.firstinspires.ftc.teamcode.robot.boats;

import static java.lang.Math.E;
import static java.lang.Math.abs;

import android.content.Context;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.chassis.Meccanum.Meccanum;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Config
public class Bert  extends Meccanum implements Robot {
    protected HardwareMap hw = null;

    public boolean opModeIsActive = true;
    public boolean pidActive = false;

    SlideThread st = new SlideThread();

    DcMotor slides, rightHang, leftHang, spinners;
    public SampleMecanumDrive rr = null;
    private Servo claw, leftSlide, rightSlide, tilt, plane;
    Telemetry tele = FtcDashboard.getInstance().getTelemetry();
    public void teleinit(HardwareMap hardwareMap) {
        // internal IMU setup (copied and pasted, idk what it really does, but it works)
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void resetImu() {
        this.offset = -imu.getAngularOrientation().firstAngle;
    }
    @Override
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap);
        // init the class, declare all the sensors and motors and stuff
        // should be called before using class ALWAYS

        // internal IMU setup (copied and pasted, idk what it really does, but it works)
        // BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        // imu = hardwareMap.get(BNO055IMU.class, "imu");
        // imu.initialize(parameters);
        // angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);


        // Meccanum Motors Definition and setting prefs

        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("motorFrontLeft"); // EH1
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("motorBackLeft"); // EH4
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("motorFrontRight"); // CH2
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("motorBackRight"); // CH0

        // Reverse the left side motors and set behaviors to stop instead of coast

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        slides = (DcMotorEx) hardwareMap.dcMotor.get("slides"); // EH3
        spinners = (DcMotorEx) hardwareMap.dcMotor.get("spinners"); // CH3
        rightHang = (DcMotorEx) hardwareMap.dcMotor.get("rightHang"); // CH1
        leftHang = (DcMotorEx) hardwareMap.dcMotor.get("leftHang"); // EH0

        claw = hardwareMap.servo.get("claw");
        leftSlide = hardwareMap.servo.get("leftSlide");
        rightSlide = hardwareMap.servo.get("rightSlide");
        tilt = hardwareMap.servo.get("tilt");
        plane = hardwareMap.servo.get("plane");


        // Reverse the left side motors and set behaviors to stop instead of coast

        rightHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHang.setDirection(DcMotor.Direction.REVERSE);
        leftHang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        rr = new SampleMecanumDrive(hardwareMap);
        st.start();


        // define hw as the hardware map for possible access later in this class
        hw = hardwareMap;



        runtime.reset();
    }
    // This robot shouldnt require a claw/arm/wrist "thread" class beacuse we'll be smart and engineer it so it cant break itself
    public static double clawSlideThreshold = 800;
    public static double clawOpenPos = 0.8;
    public static double clawClosedPos = 0.34;
    public static double planeLauncedPos = 0.8;
    public static double planeReadyPos = 0;
    public static double tiltSlideThreshold = 800;
    public static double tiltPlacePos = 0.43;
    public static double tiltPickupPos = 0.31;
    public static double armPickupPos = 0.91;
    public static double armSlideThreshold = 800;
    public static double armPlacePos = 0.45;
    public static double slidePlacePos = 1600;
    public static double slidePickupPos = 0;

    public boolean getClawOpen() {
        return claw.getPosition() == clawOpenPos;
    }
    public boolean getPlaneLaunched() {
        return plane.getPosition() == planeLauncedPos;
    }
    public double getArmPos() {
        return rightSlide.getPosition();
    }
    public boolean getArmPickup() {
        return getArmPos() == armPickupPos;
    }
    public double getTiltPos() {
        return tilt.getPosition();
    }
    public boolean getTiltPickup() {
        return getTiltPos() == tiltPickupPos;
    }
    public double getSlidePos() {
        return st.pos;
    }
    public void setPlaneLaunched(boolean launched) {
        plane.setPosition(launched ? planeLauncedPos : planeReadyPos); // NEED TO TEST VALS
    }
    public void incrementTilt(double increment) {
        tilt.setPosition(tilt.getPosition() + increment);
    }
    public void setTilt(double position) {
        if (st.pos < tiltSlideThreshold && cawtFailsafe) return;
        tilt.setPosition(position * 0.96); // thats the max without stalling on the bucket
    }
    public void setClawOpen(boolean open) {
        if (!open && st.pos < clawSlideThreshold && cawtFailsafe) return;
        claw.setPosition(open ? clawOpenPos : clawClosedPos); // NEED TO TEST FOR VALS
    }
    private void setTiltUNSAFE(double position) {
        tilt.setPosition(position * 0.96); // thats the max without stalling on the bucket
    }
    private void setClawOpenUNSAFE(boolean open) {
        claw.setPosition(open ? clawOpenPos : clawClosedPos); // NEED TO TEST FOR VALS
    }
    private void setArmUNSAFE(double position) {
        leftSlide.setPosition(1-position);
        rightSlide.setPosition(position);
    }
    public void setArm(double position) {
        if (st.pos < armSlideThreshold && cawtFailsafe) return;
        leftSlide.setPosition(1-position);
        rightSlide.setPosition(position);
    }
    public void setArmPickup(boolean pickup) {
        if (st.pos < armSlideThreshold && cawtFailsafe) return;
        leftSlide.setPosition(pickup ? 1-armPickupPos : 1-armPlacePos);
        rightSlide.setPosition(pickup ? armPickupPos : armPlacePos);
    }
    public void setTiltPickup(boolean pickup) {
        if (st.pos < tiltSlideThreshold && cawtFailsafe) return;
        tilt.setPosition(pickup ? tiltPickupPos : tiltPlacePos);
    }
    public void spintake(double power) {
        spinners.setPower(power);
    }
    public void driveHangs(double power) {
        rightHang.setPower(power);
        leftHang.setPower(power);
    }
    public void driveLeftHang(double power) {
        leftHang.setPower(power);
    }
    public void driveRightHang(double power) {
        rightHang.setPower(power);
    }


    public void playSound(String filename){
        // play a sound
        // doesnt work but would be really fun :(

        int startupID = hw.appContext.getResources().getIdentifier(filename, "raw", hw.appContext.getPackageName());
        Context appContext = hw.appContext;
        SoundPlayer.getInstance().startPlaying(appContext, startupID);
    }
    public Orientation getAngularOrientation(){
        return imu.getAngularOrientation();
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motorBackLeft.setZeroPowerBehavior(zeroPowerBehavior);
        motorBackRight.setZeroPowerBehavior(zeroPowerBehavior);
        motorFrontRight.setZeroPowerBehavior(zeroPowerBehavior);
        motorFrontLeft.setZeroPowerBehavior(zeroPowerBehavior);
    }
    public void tick() {
        st.tick();
        checkCatPos();
        //cawt.tick();
    }
    public void checkCatPos() {
        if (st.pos < clawSlideThreshold && !getClawOpen()) setClawOpenUNSAFE(true);
        if (st.pos < tiltSlideThreshold && !getTiltPickup()) setTiltUNSAFE(tiltPickupPos);
        if (st.pos < armSlideThreshold && !getArmPickup()) setArmUNSAFE(armPickupPos);
    }

    public boolean armActive() {
        return st.pos > armSlideThreshold;
    }
    public boolean tiltActive() {
        return st.pos > tiltSlideThreshold;
    }
    public boolean clawActive() {
        return st.pos > clawSlideThreshold;
    }
    public void autoTick() {
        st.tick();
        if (cawtFailsafe) checkCatPos();
        rr.update();
    }

    public boolean SLIDE_TARGETING = false;
    //SLIDES
    public void driveSlides(double power) {
        st.driveSlides(power);
    }
    public void setSlideTarget(double target) {
        st.setTarget(target);
    }
    public void setSlideTargeting(boolean targeting) {
        st.setTargeting(targeting);
    }
    // CAWT (Claw Arm Wrist Thread)
    /*
    public void setArmTarget(double target) {
        cawt.setArmTarget(target);
    }
    public void setWristTarget(double target) {
        cawt.setWristTarget(target);
    }
    public void setClawTarget(double target) {
        cawt.setClawTarget(target);
    }

    public boolean isBusy() {
        // true if there are any unresolved targets in caw or slide
        return cawt.isBusy() && st.isBusy();
    }
    private class ClawArmWristThread {
        public double clawClosed = 0.7; // 0.76 for non-straining I think
        public double clawOpen = 1;
        public double levelWristPlace = 0.05;
        // will we ever want a non level claw rot for cycling back from a place? or does that just complicate too much :/
        public double levelWristPickup = 0.725;
        public double lowArmPickup = 0.92;
        public double levelArmPickup = 0.915;
        public double upSlantArmPlace = 0.31;
        public double levelArmPlace = 0.24;
        public double beforeSlidesArmPlace = 0.53;
        public double beforeSlidesArmPickup = 0.63;

        // TODO: should make function out of needed values so for some arbitrary clawPos I can extrapolate maxClawBeforeSlidesDistance for Pickup/Placea
        // TODO: NEEDED VALUES: how close the claw hits the slides (each side) if closed and rotating
        public double maxClawClosedBeforeSlidesDistancePickup = 0.69; // GUESS
        public double maxClawClosedBeforeSlidesDistancePlace = 0.47; // GUESS
        // TODO: NV: how close claw hits slides (each side) when open and rotating
        public double maxClawOpenBeforeSlidesDistancePickup = 0.8; // GUESS
        public double maxClawOpenBeforeSlidesDistancePlace = 0.36; // GUESS

        public double armTarget = lowArmPickup;
        public double wristTarget = levelWristPickup;
        public double clawTarget = clawOpen;

        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        public void setTele(Telemetry t) {
            tele = t;
        }

        public void start() {

        }

        public void tick() {
            setArmPos(armTarget);
            setWristPos(wristTarget);
            setClawPos(clawTarget);
        }
        public void setArmTarget(double target) {
            armTarget = target;
        }
        public void setWristTarget(double target) {
            armTarget = target;
        }
        public void setClawTarget(double target) {
            clawTarget = target;
        }
        // second priority (shoudnt ever conflict tho)
        boolean CLAW_SAFE = false; // never assume safety until checked, dont leave room for nullpointers
        public void setClawPos(double pos) {
            double buffer = 0.03;
            CLAW_SAFE = beforeSlidesArmPlace - buffer < getArmPos() && getArmPos() < beforeSlidesArmPickup + buffer;
            if (CLAW_SAFE) claw.setPosition(pos);
            else claw.setPosition(clawClosed);

        }
        // third priority
        boolean WRIST_SAFE = false; // never assume safety until checked, dont leave room for nullpointers
        public void setWristPos(double pos) {
            if (WRIST_SAFE) wrist.setPosition(pos);
            else wrist.setPosition(abs(pos - levelWristPickup) > abs(pos - levelWristPlace) ? levelWristPlace : levelWristPickup);
        }

        boolean ARM_THROUGH_SAFE = false; // never assume safety until checked, dont leave room for nullpointers
        boolean ARM_THROUGH_ON_CURRENT = false; // never assume safety until checked, dont leave room for nullpointers
        public void setArmPos(double pos) {
            double buffer = 0.1; // safety buffer
            ARM_THROUGH_SAFE = getWristPos() == levelWristPickup || getWristPos() == levelWristPlace;
            ARM_THROUGH_ON_CURRENT = (getArmPos() < beforeSlidesArmPlace - buffer && pos > beforeSlidesArmPlace - buffer)
                    || (getArmPos() > beforeSlidesArmPickup + buffer && pos < beforeSlidesArmPickup + buffer);
            if (ARM_THROUGH_SAFE) {
                // if wrist is in non-obstruc pos (with cone), ARM_THROUGH_SAFE, dont care, set arm to whatever
                setArmPosBasic(pos);
            }
            if (ARM_THROUGH_ON_CURRENT && !ARM_THROUGH_SAFE) {
                // if current movement will pass through slides (ARM_THROUGH_ON_CURRENT),
                //      and wrist in possibly obstructive place (!ARM_THROUGH_SAFE),
                //      freeze and rotate wrist first
                WRIST_SAFE = false;
                setArmPosBasic(getArmPos()); // freeze arm until safe
            }
            // otherwise, wrist is safe to do whatever
            else WRIST_SAFE = true;
        }
        private void setArmPosBasic(double pos) {
            // servos are slightly off, this corrects.
            // NOTE: NEVER DIRECTLY SET ARM SERVO POSITIONS WITHOUT THIS FUNCTION!!!!
            rightArm.setPosition(pos > 0.03 ? pos : 0);
            leftArm.setPosition(pos-0.03 > 0 ? pos-0.03 : 0);
        }
        public double getArmPos() {
            return rightArm.getPosition();
        }
        public double getClawPos() {
            return claw.getPosition();
        }
        public double getWristPos() {
            return wrist.getPosition();
        }
        public boolean isBusy() {
            // claw has leway because sometimes set to impossible position in order to squeeze
            double clawBuffer = 0.08;
            boolean armBusy = armTarget == getArmPos();
            boolean clawBusy = (getClawPos() - clawBuffer) < clawTarget && (getClawPos() + clawBuffer) > clawTarget;
            // !WRIST_SAFE to avoid stalling in a set position for arm in slides while setpos for wrist is weird
            boolean wristBusy = getWristPos() == wristTarget || !WRIST_SAFE;
            return armBusy && clawBusy && wristBusy;
        }
    }
*/
    public static double maxHeight = 2800;//2848 technically
    public static double minHeight = -80; // cprobably good (maybe just set to 20 so the 10ish off errors are unnoticed)

    public static double scaler = 0.008; // scales width of sigmoid, a const goes along with it so dont change on its own
    public static double sp = 0.003; // slide kp const
    public double slideTar = 0; // target of slide (duh)
    public boolean cawtFailsafe = true;
    public void setCawtFailsafe(boolean on) {
        cawtFailsafe = on;
    }
    public void setCawtThresholds(double threshold) {
        tiltSlideThreshold = threshold;
        armSlideThreshold = threshold;
        clawSlideThreshold = threshold;
    }
    public void setDownCorrection(boolean corrected) {
        st.downCorrection = corrected;
    }
    public void setDownCorrectionFactor(double factor) {
        st.correctionFactor = factor;
    }
    public void resetSlides() {
        st.resetSlideBasePos();
    }

    private class SlideThread {
        public boolean downCorrection = false;
        public double correctionFactor = 0.25;
        public double basePos = 0;
        public double pos = 0;
        public double errorThreshold = 20;
        public double derivativeThreshold = 1;

        public double power = 0;

        //public double slideTar = 0;
        public PID slidePID;

        //public double maxHeight = 1000;
        //public double minHeight = 0;

        //public double differenceScalar = 0.0001;
        //public double scaler = 50;
        Telemetry tele = FtcDashboard.getInstance().getTelemetry();
        public void setTele(Telemetry t) {
            tele = t;
        }

        public void start() {
            basePos = slides.getCurrentPosition();

            slidePID = new PID(0.001, 0, 0, false);
            tele = FtcDashboard.getInstance().getTelemetry();
        }

        public void tick() {

            slidePID.setConsts(sp, 0, 0);
            slidePID.setTarget(slideTar);
            pos = -(slides.getCurrentPosition() - basePos);

            tele.addData("pos", pos);
            tele.addData("targeting", SLIDE_TARGETING);
            tele.addData("slidetar", slideTar);
            tele.addData("slidep", sp);

            if (pos < minHeight  && power < 0) {
                SLIDE_TARGETING = true;
                slideTar = minHeight;
            }
            if (pos > maxHeight && power > 0) {
                SLIDE_TARGETING = true;
                slideTar = maxHeight;
            }
            if (SLIDE_TARGETING) {
                power = -slidePID.tick(pos);
                tele.addData("pidpower", power);
            }

            tele.addData("drivingPower", minMaxScaler(pos, power));
            tele.update();

            slides.setPower(minMaxScaler(pos, power));
        }

        public double minMaxScaler(double x, double power) {
            double p = power * (power > 0 ? ((1.3 * 1/(1+Math.pow(E, -scaler*(x-300+minHeight)))) - 0.1) : ((1.3 * 1/(1+Math.pow(E, scaler*(x+300-maxHeight)))) - 0.1));
            if (p > -0.05 && p < 0.25 && pos < 400 && downCorrection) { // let it go farther down
                p = st.pos > 0 ? correctionFactor : 0.15;
            }
            return p;
        }

        public void driveSlides(double p) {
            //if (p == 0) setTarget(pos); // untested
            tele.addData("ipower", p);
            tele.addData("cpower", power);
            tele.update();
            SLIDE_TARGETING = false;
            power = -p;
        }
        public void setTargeting(boolean targeting) {
            SLIDE_TARGETING = targeting;
        }

        public void setTarget(double tar) {
            slideTar = tar;
            SLIDE_TARGETING = true;
        }
        public void resetSlideBasePos() {
            slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            basePos = slides.getCurrentPosition();
        }
        public boolean isBusy() {

            return slidePID.getDerivative() < derivativeThreshold && abs(pos - slideTar) < errorThreshold;
            //                                                       could get proportion (^) from pid but dont want to
        }

    }
}
