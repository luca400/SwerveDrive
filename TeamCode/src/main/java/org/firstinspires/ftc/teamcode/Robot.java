package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.drivetrain.Localization.Localizer;
import org.firstinspires.ftc.teamcode.drivetrain.auto.P2PController;
import org.firstinspires.ftc.teamcode.drivetrain.drive.Follower;
import org.firstinspires.ftc.teamcode.drivetrain.drive.SwerveDrivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.geometry.Pose;

public class Robot {

    public final RobotHardware hardware;
    public final Localizer localizer;
    public final SwerveDrivetrain drivetrain;
    public final Follower follower;
    public final P2PController p2p;

    public Robot() {

        hardware = RobotHardware.getInstance();

        drivetrain = new SwerveDrivetrain(hardware);
        localizer = new Localizer(hardware.pinpoint);
        follower = new Follower(drivetrain);
        p2p = new P2PController(follower);
    }

    /* ------------------------------------------------
       Mode switching
       ------------------------------------------------ */

    public void initTeleOp() {
        follower.setMode(Follower.Mode.TELEOP);
        follower.resetHeadingTarget(localizer.getPose().heading);
    }

    public void initAuto(Pose startPose) {
        localizer.reset(startPose);
        follower.setMode(Follower.Mode.AUTO);
        follower.resetHeadingTarget(startPose.heading);
    }

    /* ------------------------------------------------
       Periodic helpers
       ------------------------------------------------ */

    public void updateLocalization() {
        localizer.update();
    }

    public Pose getPose() {
        return localizer.getPose();
    }

    /* ------------------------------------------------
       Shutdown (IMPORTANT with singleton hardware)
       ------------------------------------------------ */

    public void shutdown() {
        hardware.enabled = false;
    }
}
