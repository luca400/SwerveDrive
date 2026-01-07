package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drivetrain.geometry.Pose;

@TeleOp(name = "Swerve Test TeleOp", group = "Test")
public class SwerveTest extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot();
        robot.initTeleOp();

        waitForStart();

        while (opModeIsActive()) {
            robot.updateLocalization();
            Pose pose = robot.getPose();

            double x = -gamepad1.left_stick_y;   // forward
            double y =  gamepad1.left_stick_x;   // strafe
            double rot = gamepad1.right_stick_x; // rotate
            robot.follower.update(
                    x,
                    y,
                    rot,
                    pose.heading
            );
            telemetry.addData("x", "%.2f", pose.x);
            telemetry.addData("y", "%.2f", pose.y);
            telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(pose.heading));
            telemetry.update();
        }

        robot.shutdown();
    }
}

