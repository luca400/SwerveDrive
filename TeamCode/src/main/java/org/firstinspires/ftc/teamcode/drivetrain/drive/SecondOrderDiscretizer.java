package org.firstinspires.ftc.teamcode.drivetrain.drive;

public class SecondOrderDiscretizer {

    public static ChassisSpeeds discretize(ChassisSpeeds s, double dt) {
        double vx = s.vx;
        double vy = s.vy;
        double omega = s.omega;

        if (Math.abs(omega) < 1e-6) {
            return s;
        }

        double sin = Math.sin(omega * dt);
        double cos = Math.cos(omega * dt);

        double sTerm = sin / (omega * dt);
        double cTerm = (1 - cos) / (omega * dt);

        double vxCorrected = vx * sTerm - vy * cTerm;
        double vyCorrected = vx * cTerm + vy * sTerm;

        return new ChassisSpeeds(vxCorrected, vyCorrected, omega);
    }
}

