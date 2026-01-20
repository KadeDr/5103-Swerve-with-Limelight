package frc.robot.commands.limelight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants.PositionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.subsystems.DriveSubsystem;

public class FollowAlgaeCommand extends Command {
    String targetName;
    String limelightName;

    DriveSubsystem robotDrive;

    public FollowAlgaeCommand(String targetName, String limelightName, DriveSubsystem robotDrive) {
        this.targetName = targetName;
        this.limelightName = limelightName;
        this.robotDrive = robotDrive;
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(limelightName, 1);
    }

    @Override
    public void execute() {
        LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
        if (results.targets_Detector.length <= 0) {
            return;
        }

        LimelightTarget_Detector detection = results.targets_Detector[0];
        String className = detection.className;
        double confidence = detection.confidence;
        double area = detection.ta;

        if (className != targetName) {
            return;
        }

        double ta = detection.ta;
        double tx = detection.tx;

        double currentheading = robotDrive.getHeading();
        double targetHeading = 0.0;

        double desiredArea = 3.0;

        double kP_Distance = PositionConstants.kP_Distance;
        double kP_Strafe = PositionConstants.kP_Strafe;
        double kP_Gyro = PositionConstants.kP_Rot;
        double maxSpeed = 0.4;
        double maxRotationSpeed = 0.2;

        double driveError = desiredArea - ta;
        double forwardSpeed = driveError * kP_Distance;

        double headingError = Math.IEEEremainder(targetHeading - currentheading, 360);
        double rotationSpeed = headingError * kP_Gyro;

        double strafeSpeed = -tx * kP_Strafe;

        forwardSpeed = MathUtil.clamp(forwardSpeed, -maxSpeed, maxSpeed);
        rotationSpeed = MathUtil.clamp(rotationSpeed, -maxRotationSpeed, maxRotationSpeed);
        strafeSpeed = MathUtil.clamp(strafeSpeed, -maxSpeed, maxSpeed);

        robotDrive.drive(forwardSpeed, strafeSpeed, rotationSpeed, false, 0);
    }
}
