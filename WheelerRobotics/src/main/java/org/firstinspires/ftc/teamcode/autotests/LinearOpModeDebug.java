package org.firstinspires.ftc.teamcode.autotests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Base class for user defined linear operation modes (op modes).
 * <p>
 * This class derives from OpMode, but you should not override the methods from
 * OpMode.
 */
@SuppressWarnings("unused")
public abstract class LinearOpModeDebug extends LinearOpMode {

    //------------------------------------------------------------------------------------------------
    // State
    //------------------------------------------------------------------------------------------------

    private volatile boolean  userMethodReturned = false;
    private volatile boolean  userMonitoredForStart = false;
    private final Object      runningNotifier = new Object();

    //------------------------------------------------------------------------------------------------
    // Construction
    //------------------------------------------------------------------------------------------------

    public LinearOpModeDebug() {
    }

    //------------------------------------------------------------------------------------------------
    // Operations
    //------------------------------------------------------------------------------------------------

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     * @throws InterruptedException
     */
    abstract public void runOpMode() throws InterruptedException;

    final void internalRunOpMode() throws InterruptedException {
        // Do NOT call super.internalRunOpMode().

        // We need to reset these fields because BlocksOpMode (which is a subclass of LinearOpMode)
        // instances are re-used.
        userMethodReturned = false;
        userMonitoredForStart = false;

        try {
            runOpMode();
        }
        catch (Exception e) {
            FtcDashboard.getInstance().getTelemetry().addData("Error!", e.getMessage());
            FtcDashboard.getInstance().getTelemetry().log();
            FtcDashboard.getInstance().getTelemetry().update();
        }
        userMethodReturned = true;
        RobotLog.d("User runOpModeMethod exited");
        requestOpModeStop();
    }
}
