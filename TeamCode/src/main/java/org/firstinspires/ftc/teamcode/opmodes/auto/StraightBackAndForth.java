package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drivetrain.drive.Follower;
import org.firstinspires.ftc.teamcode.drivetrain.geometry.Pose;

@Autonomous(name = "P2P Straight Test", group = "Test")
public class StraightBackAndForth extends LinearOpMode {

    private Robot robot;
    public static double TARGET_X = 24.0;
    public static double TARGET_Y = 0.0;
    public static double TARGET_HEADING = 0.0;
    private enum State {
        FORWARD,
        BACK
    }

    private State state = State.FORWARD;

    @Override
    public void runOpMode() {

        robot = new Robot();

        // Start pose explicitly set to (0,0,0)
        robot.initAuto(new Pose(0, 0, 0));
        waitForStart();

        while (opModeIsActive()) {
            robot.updateLocalization();
            Pose current = robot.getPose();

            Pose target;
            if (state == State.FORWARD) {
                target = new Pose(
                        TARGET_X,
                        TARGET_Y,
                        TARGET_HEADING
                );
            } else {
                target = new Pose(
                        0.0,
                        0.0,
                        TARGET_HEADING
                );
            }

            robot.p2p.update(current, target);

            if (robot.p2p.atTarget(current, target)) {
                state = (state == State.FORWARD)
                        ? State.BACK
                        : State.FORWARD;
            }
            telemetry.addData("State", state);
            telemetry.addData("Target",
                    "x=%.1f y=%.1f h=%.1f°",
                    target.x,
                    target.y,
                    Math.toDegrees(target.heading)
            );
            telemetry.addData("Pose",
                    "x=%.1f y=%.1f h=%.1f°",
                    current.x,
                    current.y,
                    Math.toDegrees(current.heading)
            );
            telemetry.update();
        }

        robot.shutdown();
    }
}
