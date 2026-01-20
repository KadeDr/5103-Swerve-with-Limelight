package frc.robot.commands.limelight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants.PositionConstants;
import frc.robot.subsystems.DriveSubsystem;

public class PositionControlCommand extends Command {
    private final int aprilTagId;

    private final XboxController controller;

    private final String limelightName;

    private final DriveSubsystem robotDrive;

    /**
     * 
     * @param aprilTagId    The Id for the desired april tag target
     * @param controller    The xbox controller
     * @param limelightName The name of the limelight assigned on the limelight
     *                      webpage
     * @param robotDrive    The robot drive subsystem
     */
    public PositionControlCommand(int aprilTagId, XboxController controller, String limelightName,
            DriveSubsystem robotDrive) {
        this.aprilTagId = aprilTagId;
        this.controller = controller;
        this.limelightName = limelightName;
        this.robotDrive = robotDrive;
        addRequirements(robotDrive);
    }

    /**
     * 
     * @param aprilTagId    The Id for the desired april tag target
     * @param controller    The xbox controller
     * @param limelightName The name of the limelight assigned on the limelight
     *                      webpage
     * @param robotDrive    The robot drive subsystem
     */
    public PositionControlCommand(int aprilTagId, CommandXboxController controller, String limelightName,
            DriveSubsystem robotDrive) {
        this(aprilTagId, controller.getHID(), limelightName, robotDrive);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(limelightName, 0);
        LimelightHelpers.setPriorityTagID(limelightName, aprilTagId);
    }

    @Override
    public void execute() {
        boolean hasTarget = LimelightHelpers.getTV(limelightName);

        if (!hasTarget) {
            System.err.println("No target!");
            robotDrive.drive(0, 0, 0, false, 0);

            double tx = LimelightHelpers.getTX(limelightName);
            double ta = LimelightHelpers.getTA(limelightName);
            double currentTagId = LimelightHelpers.getFiducialID(limelightName);

            if (currentTagId != aprilTagId) {
                return;
            }

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

            if (hasTarget && Math.abs(tx) < desiredArea) {
                controller.setRumble(RumbleType.kBothRumble, 0.1);
            } else {
                controller.setRumble(RumbleType.kBothRumble, 0);
            }

            robotDrive.drive(forwardSpeed, strafeSpeed, rotationSpeed, false, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(RumbleType.kBothRumble, 0);
        LimelightHelpers.setPriorityTagID(limelightName, -1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
