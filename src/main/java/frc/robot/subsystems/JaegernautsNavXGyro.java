package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

public final class JaegernautsNavXGyro extends AHRS {
    private static volatile JaegernautsNavXGyro instance;

    // Checks if the singleton exists, if it doesn't it creates one, else, it uses the already existing one
    public static final synchronized JaegernautsNavXGyro getInstance() {
        return instance == null ? instance = new JaegernautsNavXGyro() : instance;
    }

    // The offset of the gyro
    // If your gyro isn't driving straight, adjust the offset till it does
    private double angleOffset = 0;

    // Gets the gyro's port
    private JaegernautsNavXGyro() {
        super(NavXComType.kMXP_SPI);
    }

    public final void setOffsetCW(final Rotation2d offset) {
        // Adjusts the angleOffset for a clockwise rotation
        // Calculates the adjusted offset based on the gyro's current heading and the input value
        this.angleOffset = calculateOffsetCW(offset.getDegrees());
    }

    public final void setOffsetCCW(final Rotation2d offset) {
        // Same as setoffsetCW, but counterclockwise instead
        this.angleOffset = 360 - calculateOffsetCW(offset.getDegrees());
    }

    // Converts angleOffset to degrees
    public final Rotation2d getAngleOffsetCW() { return Rotation2d.fromDegrees(angleOffset); }

    // Same as getAngleOffsetCW, but negated to get counterclockwise
    public final Rotation2d getAngleOffsetCCW() { return Rotation2d.fromDegrees(angleOffset).times(-1); }

    // Retrieves the robots current heading (direction) based on the current gyro readings
    public final Rotation2d getHeadingCW() { return Rotation2d.fromDegrees(getDegreesClockwise()); }

    // Same as above, but counterclockwise instead
    public final Rotation2d getHeadingCCW() { return Rotation2d.fromDegrees(getDegreesCounterClockwise()); }

    // Gets the angular velocity.. in other words, gets how fast the robot is rotating
    public final Rotation2d getAngularVelocity() { return Rotation2d.fromDegrees(getRate()); }

    // Calculates the angle offset needed to adjust the gyro's heading to match the input offset value
    private final double calculateOffsetCW(final double offset) {
        return (offset - getFusedHeading() + 360) % 360; // % 360 makes sure the values reset to 0 after it reaches 360
    }

    // Adds the angle offset to the gyro reading
    private final double getDegreesClockwise() { return (getFusedHeading() + angleOffset) % 360; }

    // Same thing as above, but counterclockwise reading instead
    private final double getDegreesCounterClockwise() { return 360 - getDegreesClockwise(); }
}