package org.firstinspires.ftc.teamcode.drivetrain.drive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.AbsoluteAnalogEncoder;

import java.util.Locale;

public class SwerveModule {

    /* ---------------- Hardware ---------------- */

    private final DcMotorEx driveMotor;
    private final CRServo steerServo;
    private final AbsoluteAnalogEncoder steerEncoder;

    /* ---------------- Control ---------------- */

    private final PIDFController steerController;

    public static double P = 0.6, I = 0.0, D = 0.1;
    public static double K_STATIC = 0.03;

    public static double MAX_SERVO = 1.0;
    public static double MAX_MOTOR = 1.0;

    public static boolean MOTOR_FLIPPING = true;

    /* ---------------- Drive constants ---------------- */

    public static double WHEEL_RADIUS = 1.4; // inches
    public static double GEAR_RATIO = 1 / (3.5 * 1.5 * 2);
    public static final double TICKS_PER_REV = 28;

    /* ---------------- State ---------------- */

    private double targetAngle = 0.0;   // commanded angle (rad)
    private double currentAngle = 0.0;  // continuous angle (rad)

    private double lastRawAngle = 0.0;
    private double continuousAngle = 0.0;

    public boolean wheelFlipped = false;
    public double lastMotorPower = 0.0;

    /* ---------------- Constructor ---------------- */

    public SwerveModule(DcMotorEx drive, CRServo steer, AbsoluteAnalogEncoder encoder) {
        driveMotor = drive;
        steerServo = steer;
        steerEncoder = encoder;

        MotorConfigurationType motorConfigurationType = driveMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(MAX_MOTOR);
        driveMotor.setMotorType(motorConfigurationType);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ((CRServoImplEx) steerServo)
                .setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));

        steerController = new PIDFController(P, I, D, 0);
    }

    public SwerveModule(HardwareMap hw, String mName, String sName, String eName) {
        this(
                hw.get(DcMotorEx.class, mName),
                hw.get(CRServo.class, sName),
                new AbsoluteAnalogEncoder(hw.get(AnalogInput.class, eName))
        );
    }

    /* ---------------- Sensor Read ---------------- */

    public void read() {
        double raw = steerEncoder.getCurrentPosition(); // assumed wrapped

        double delta = raw - lastRawAngle;
        if (delta > Math.PI)  delta -= 2 * Math.PI;
        if (delta < -Math.PI) delta += 2 * Math.PI;

        continuousAngle += delta;
        lastRawAngle = raw;
        currentAngle = continuousAngle;
    }

    /* ---------------- Control Update ---------------- */

    public void update() {
        steerController.setPIDF(P, I, D, 0);

        double optimizedTarget = targetAngle;
        double error = normalizeRadians(optimizedTarget - currentAngle);

        /* ---- Wheel flipping (NO target mutation) ---- */
        if (MOTOR_FLIPPING && Math.abs(error) > Math.PI / 2.0) {
            optimizedTarget = normalizeRadians(targetAngle - Math.PI);
            wheelFlipped = true;
        } else {
            wheelFlipped = false;
        }

        error = normalizeRadians(optimizedTarget - currentAngle);

        /* ---- Steering control ---- */
        double output =
                P * error
                        + (Math.abs(error) > 0.02 ? K_STATIC * Math.signum(error) : 0.0);

        output = Range.clip(output, -MAX_SERVO, MAX_SERVO);
        if (Double.isNaN(output)) output = 0.0;

        steerServo.setPower(output);
    }

    /* ---------------- Drive ---------------- */

    public void setMotorPower(double power) {
        if (wheelFlipped) power *= -1.0;
        lastMotorPower = power;
        driveMotor.setPower(power);
    }

    /* ---------------- Commands ---------------- */

    public void setTargetRotation(double angleRad) {
        targetAngle = normalizeRadians(angleRad);
    }

    /* ---------------- Accessors ---------------- */

    public double getModuleRotation() {
        return currentAngle;
    }

    public double getTargetRotation() {
        return targetAngle;
    }

    public double getWheelPosition() {
        return encoderTicksToInches(driveMotor.getCurrentPosition());
    }

    public double getWheelVelocity() {
        return encoderTicksToInches(driveMotor.getVelocity());
    }

    public double getServoPower() {
        return steerServo.getPower();
    }

    public void setMode(DcMotor.RunMode mode){
        driveMotor.setMode(mode);
    }

    /* ---------------- Telemetry ---------------- */

    public String getTelemetry(String name) {
        return String.format(
                Locale.ENGLISH,
                "%s | flipped=%b | angle=%.2f | target=%.2f | motor=%.2f",
                name, wheelFlipped, currentAngle, targetAngle, lastMotorPower
        );
    }

    /* ---------------- Utils ---------------- */

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
}
