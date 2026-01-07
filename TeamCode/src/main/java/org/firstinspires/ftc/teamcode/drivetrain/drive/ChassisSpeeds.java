package org.firstinspires.ftc.teamcode.drivetrain.drive;

public class ChassisSpeeds {
    public double vx;     // forward (+X)
    public double vy;     // left (+Y)
    public double omega;  // CCW (+)

    public ChassisSpeeds(double vx, double vy, double omega) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }
}

