package org.firstinspires.ftc.teamcode.drivetrain.drive;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;

public class SwerveKinematics {

    private final double trackWidth;
    private final double wheelBase;
    private final double R;

    public static class ModuleState {
        public double speed;
        public double angle;
    }

    public SwerveKinematics(double trackWidth, double wheelBase) {
        this.trackWidth = trackWidth;
        this.wheelBase = wheelBase;
        this.R = hypot(trackWidth, wheelBase);
    }

    public void toModuleStates(
            double vx,
            double vy,
            double omega,
            ModuleState[] out
    ) {
        double a = vx - omega * (wheelBase / R);
        double b = vx + omega * (wheelBase / R);
        double c = vy - omega * (trackWidth / R);
        double d = vy + omega * (trackWidth / R);

        // FL
        out[0].speed = hypot(b, c);
        out[0].angle = atan2(b, c);

        // FR
        out[1].speed = hypot(b, d);
        out[1].angle = atan2(b, d);

        // BR
        out[2].speed = hypot(a, d);
        out[2].angle = atan2(a, d);

        // BL
        out[3].speed = hypot(a, c);
        out[3].angle = atan2(a, c);
    }

    public void desaturate(ModuleState[] states) {
        double max = 0.0;
        for (ModuleState s : states) max = Math.max(max, Math.abs(s.speed));
        if (max <= 1.0) return;
        for (ModuleState s : states) s.speed /= max;
    }
}

