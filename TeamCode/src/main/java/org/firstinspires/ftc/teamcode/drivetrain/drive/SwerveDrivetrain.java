package org.firstinspires.ftc.teamcode.drivetrain.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.common.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.drivetrain.geometry.Pose;
import org.firstinspires.ftc.teamcode.drivetrain.geometry.MathUtils;

public class SwerveDrivetrain {

    public static double TRACK_WIDTH = 9;
    public static double WHEEL_BASE  = 9;

    private final SwerveModule[] modules;
    private final SwerveKinematics kinematics;
    private final SwerveKinematics.ModuleState[] states;

    private boolean locked = false;
    private final double minPow = 0.1;
    private boolean USE_WHEEL_FEEDFORWARD = false;

    public SwerveDrivetrain(RobotHardware robot) {

        modules = new SwerveModule[]{
                new SwerveModule(robot.frontLeftMotor,  robot.frontLeftServo,
                        new AbsoluteAnalogEncoder(robot.frontLeftEncoder, 3.3).setInverted(true)),
                new SwerveModule(robot.frontRightMotor, robot.frontRightServo,
                        new AbsoluteAnalogEncoder(robot.frontRightEncoder, 3.3).setInverted(true)),
                new SwerveModule(robot.backRightMotor,  robot.backRightServo,
                        new AbsoluteAnalogEncoder(robot.backRightEncoder, 3.3).setInverted(true)),
                new SwerveModule(robot.backLeftMotor,   robot.backLeftServo,
                        new AbsoluteAnalogEncoder(robot.backLeftEncoder, 3.3).setInverted(true))
        };

        for (SwerveModule m : modules)
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        kinematics = new SwerveKinematics(TRACK_WIDTH, WHEEL_BASE);

        states = new SwerveKinematics.ModuleState[4];
        for (int i = 0; i < 4; i++)
            states[i] = new SwerveKinematics.ModuleState();
    }

    /* ----------- Loop phases ----------- */

    public void read() {
        for (SwerveModule m : modules) m.read();
    }

    /**
     * pose.x       -> vx
     * pose.y       -> vy
     * pose.heading -> omega
     */
    public void set(Pose pose) {

        if (locked) {
            for (int i = 0; i < 4; i++) {
                states[i].speed = 0;
                states[i].angle = (i % 2 == 0) ? Math.PI / 4 : -Math.PI / 4;
            }
            return;
        }

        kinematics.toModuleStates(
                pose.x,
                pose.y,
                pose.heading,
                states
        );

        kinematics.desaturate(states);
    }

    public void write() {
        for (int i = 0; i < 4; i++) {
            double speed = states[i].speed;
            double angle = MathUtils.norm(states[i].angle);

            double driveCmd =
                    Math.abs(speed) +
                            (USE_WHEEL_FEEDFORWARD ? minPow * Math.signum(speed) : 0);

            modules[i].setMotorPower(driveCmd);
            modules[i].setTargetRotation(angle);
        }
    }

    public void updateModules() {
        for (SwerveModule m : modules) m.update();
    }

    /* ----------- Misc ----------- */

    public void setLocked(boolean locked) {
        this.locked = locked;
    }

    public int getModuleCount() {
        return modules.length;
    }

    public void applyModuleState(int index, double speed, double angle) {
        modules[index].setMotorPower(speed);
        modules[index].setTargetRotation(angle);
    }

    public double getComputedAngle(int index) {
        return states[index].angle;
    }
}
