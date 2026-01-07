package org.firstinspires.ftc.teamcode.drivetrain.auto;

import com.arcrobotics.ftclib.controller.PIDFController;
import org.firstinspires.ftc.teamcode.drivetrain.drive.Follower;
import org.firstinspires.ftc.teamcode.drivetrain.geometry.Pose;

public class P2PController {

    private final Follower follower;

    /* ============================================================
       TUNABLE PID GAINS
       ============================================================ */

    public static double kP_X = 0.08;
    public static double kI_X = 0.0;
    public static double kD_X = 0.0;

    public static double kP_Y = 0.08;
    public static double kI_Y = 0.0;
    public static double kD_Y = 0.0;

    public static double kP_THETA = 2.5;
    public static double kI_THETA = 0.0;
    public static double kD_THETA = 0.15;

    /* ============================================================
       LIMITS & TOLERANCES
       ============================================================ */

    public static double MAX_VEL = 20.0;      // inches / second
    public static double MAX_OMEGA = 3.0;     // rad / second

    public static double POS_TOL = 1.0;       // inches
    public static double HEADING_TOL = Math.toRadians(2.0);

    /* ============================================================
       SLOWDOWN PARAMETERS (NEW)
       ============================================================ */

    // Distance (inches) at which slowdown begins
    public static double SLOW_RADIUS = 10.0;

    // Minimum scaling when very close (prevents stalling)
    public static double MIN_SLOW_SCALE = 0.15;

    /* ============================================================
       PID CONTROLLERS
       ============================================================ */

    private final PIDFController xPID;
    private final PIDFController yPID;
    private final PIDFController headingPID;

    public P2PController(Follower follower) {
        this.follower = follower;

        xPID = new PIDFController(kP_X, kI_X, kD_X, 0.0);
        yPID = new PIDFController(kP_Y, kI_Y, kD_Y, 0.0);

        headingPID = new PIDFController(kP_THETA, kI_THETA, kD_THETA, 0.0);
    }

    /**
     * Drive robot toward a target pose using PID-based P2P control
     * with distance-based slowdown.
     */
    public void update(Pose current, Pose target) {

        /* ---------- Update gains (live tuning) ---------- */
        xPID.setPIDF(kP_X, kI_X, kD_X, 0.0);
        yPID.setPIDF(kP_Y, kI_Y, kD_Y, 0.0);
        headingPID.setPIDF(kP_THETA, kI_THETA, kD_THETA, 0.0);

        /* ---------- Pose error ---------- */
        double dx = target.x - current.x;
        double dy = target.y - current.y;

        double distance = Math.hypot(dx, dy);

        /* ---------- Translation PID (field-relative) ---------- */
        double vx = xPID.calculate(current.x, target.x);
        double vy = yPID.calculate(current.y, target.y);

        /* ---------- Distance-based slowdown ---------- */
        double scale = 1.0;
        if (distance < SLOW_RADIUS) {
            scale = distance / SLOW_RADIUS;
            scale = clamp(scale, MIN_SLOW_SCALE, 1.0);
        }

        vx *= scale;
        vy *= scale;

        /* ---------- Heading ---------- */
        /* ---------- Heading ---------- */
        double headingError = normalize(target.heading - current.heading);
        double omega = headingPID.calculate(0.0, headingError);


        /* ---------- Clamp outputs ---------- */
        vx = clamp(vx, MAX_VEL);
        vy = clamp(vy, MAX_VEL);
        omega = clamp(omega, MAX_OMEGA);

        /* ---------- Feed into shared follower ---------- */
        follower.update(
                vx,
                vy,
                omega,
                current.heading
        );
    }

    /* ---------- Target reached check ---------- */
    public boolean atTarget(Pose current, Pose target) {
        double posError = Math.hypot(
                target.x - current.x,
                target.y - current.y
        );

        double headingError = Math.abs(
                normalize(target.heading - current.heading)
        );

        return posError < POS_TOL && headingError < HEADING_TOL;
    }

    /* ============================================================
       UTILITIES
       ============================================================ */

    private static double clamp(double v, double max) {
        return Math.max(-max, Math.min(max, v));
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private static double normalize(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }
}
