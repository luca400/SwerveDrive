package org.firstinspires.ftc.teamcode.drivetrain.Localization;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drivetrain.geometry.Pose;

public class Localizer {

    private final GoBildaPinpointDriver pinpoint;
    private Pose pose = new Pose(0, 0, 0);

    public Localizer(GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;
    }

    /** Call ONCE per loop */
    public void update() {
        pinpoint.update();

        pose.x = pinpoint.getPosX(DistanceUnit.INCH);
        pose.y = pinpoint.getPosY(DistanceUnit.INCH);
        pose.heading = pinpoint.getHeading(AngleUnit.RADIANS);
    }

    public Pose getPose() {
        return pose;
    }

    public void reset(Pose newPose) {
        pinpoint.setPosition(
                new Pose2D(
                        DistanceUnit.INCH,
                        newPose.x,
                        newPose.y,
                        AngleUnit.RADIANS,
                        newPose.heading
                        )
        );
        pose = newPose;
    }
}

