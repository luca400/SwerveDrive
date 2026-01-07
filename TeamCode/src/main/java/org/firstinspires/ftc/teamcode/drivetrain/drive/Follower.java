package org.firstinspires.ftc.teamcode.drivetrain.drive;

import com.arcrobotics.ftclib.controller.PIDFController;
import org.firstinspires.ftc.teamcode.drivetrain.drive.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.geometry.Pose;

public class Follower {

    private final SwerveDrivetrain drivetrain;

    /* ---------- Heading hold ---------- */
    private final PIDFController headingPID;
    private double targetHeading = 0.0;

    /* ---------- Timing ---------- */
    private double lastTimeSec;

    /* ---------- Position remembrance ---------- */
    private final double[] lastAngles;
    private boolean haveLastAngles = false;

    /* ---------- Thresholds ---------- */
    private static final double TRANSLATION_EPS = 0.05;
    private static final double ROTATION_EPS = 0.05;

    /* ---------- Lock mode ---------- */
    private boolean lockMode = false;

    public enum Mode {
        AUTO,
        TELEOP
    }

    private Mode mode = Mode.TELEOP;

    public Follower(SwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        headingPID = new PIDFController(5.0, 0.0, 0.25, 0.0);

        lastTimeSec = System.nanoTime() * 1e-9;
        lastAngles = new double[drivetrain.getModuleCount()];
    }

    /**
     * Main update call.
     *
     * @param xCmd    field-relative forward command (-1..1)
     * @param yCmd    field-relative left command (-1..1)
     * @param rotCmd  rotation command (-1..1)
     * @param heading IMU heading (rad)
     */
    public void update(
            double xCmd,
            double yCmd,
            double rotCmd,
            double heading
    ) {
        /* ---------- dt ---------- */
        double now = System.nanoTime() * 1e-9;
        double dt = now - lastTimeSec;
        lastTimeSec = now;

        /* ============================================================
           STEP 1 — INPUT SHAPING
           ============================================================ */

        // Deadband
        xCmd = deadband(xCmd, 0.05);
        yCmd = deadband(yCmd, 0.05);
        rotCmd = deadband(rotCmd, 0.05);

        // Expo (softer around center)
        xCmd = expo(xCmd, 0.4);
        yCmd = expo(yCmd, 0.4);
        rotCmd = expo(rotCmd, 0.3);

        // Scaling (can be changed dynamically for slow mode)
        double speedScale = 1.0;
        double rotScale = 1.0;

        xCmd *= speedScale;
        yCmd *= speedScale;
        rotCmd *= rotScale;

        /* ============================================================
           STEP 2 — LOCK / X-STANCE OVERRIDE
           ============================================================ */
        if (lockMode) {
            drivetrain.setLocked(true);
            drivetrain.write();
            drivetrain.updateModules();
            return;
        } else {
            drivetrain.setLocked(false);
        }

        /* ============================================================
           HEADING HOLD
           ============================================================ */
        double omega;

        if (mode == Mode.AUTO) {
            // In auto, omega is trusted (P2PController provides it)
            omega = rotCmd;
        } else {
            // TELEOP heading hold
            if (Math.abs(rotCmd) < ROTATION_EPS) {
                double error = normalize(targetHeading - heading);
                omega = headingPID.calculate(0.0, error);
            } else {
                targetHeading = heading;
                omega = rotCmd;
            }
        }


        /* ============================================================
           FIELD-CENTRIC TRANSFORM
           ============================================================ */
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double vx =  xCmd * cos + yCmd * sin;
        double vy = -xCmd * sin + yCmd * cos;

        /* ============================================================
           SECOND-ORDER DISCRETIZATION
           ============================================================ */
        if (Math.abs(omega) > 1e-6) {
            double sinO = Math.sin(omega * dt);
            double cosO = Math.cos(omega * dt);

            double s = sinO / (omega * dt);
            double c = (1 - cosO) / (omega * dt);

            double vxNew = vx * s - vy * c;
            double vyNew = vx * c + vy * s;

            vx = vxNew;
            vy = vyNew;
        }

        /* ============================================================
           POSITION REMEMBRANCE
           ============================================================ */
        boolean translating = Math.hypot(vx, vy) > TRANSLATION_EPS;
        boolean rotating = Math.abs(omega) > ROTATION_EPS;

        if (!translating && !rotating && haveLastAngles) {
            // Hold module angles, zero drive
            for (int i = 0; i < drivetrain.getModuleCount(); i++) {
                drivetrain.applyModuleState(i, 0.0, lastAngles[i]);
            }
        } else {
            // Normal motion
            drivetrain.set(new Pose(vx, vy, omega));
            drivetrain.write();

            // Cache angles
            for (int i = 0; i < drivetrain.getModuleCount(); i++) {
                lastAngles[i] = drivetrain.getComputedAngle(i);
            }
            haveLastAngles = true;
        }

        drivetrain.updateModules();
    }

    /* ============================================================
       PUBLIC CONTROL
       ============================================================ */

    public void setLockMode(boolean enabled) {
        lockMode = enabled;
    }

    public void resetHeadingTarget(double heading) {
        targetHeading = heading;
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    /* ============================================================
       UTILITIES
       ============================================================ */

    private static double deadband(double x, double db) {
        return Math.abs(x) < db ? 0.0 : x;
    }

    private static double expo(double x, double e) {
        return (1 - e) * x + e * x * x * x;
    }

    private static double normalize(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
