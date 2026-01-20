package frc.robot.commands.limelight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.LimelightConstants.AimAssistConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AimAssistCommand extends Command {
    private final int aprilTagId;

    private final XboxController controller;

    private final String limelightName;

    private final DriveSubsystem robotDrive;

    private final SlewRateLimiter xSpeedLimiter;
    private final SlewRateLimiter ySpeedLimiter;
    private final SlewRateLimiter rotLimiter;

    /**
     * 
     * @param controller    The xbox controller
     * @param aprilTagId    The id for the desired april tag target
     * @param limelightName The name of the limelight assigned on the limelight
     *                      webpage
     * @param xSpeedLimiter The xSpeedLimiter
     * @param ySpeedLimiter The ySpeedLimiter
     * @param rotLimiter    The rotation limiter
     * @param robotDrive    The robot drive subsystem
     */
    public AimAssistCommand(XboxController controller, int aprilTagId, String limelightName,
            SlewRateLimiter xSpeedLimiter, SlewRateLimiter ySpeedLimiter, SlewRateLimiter rotLimiter,
            DriveSubsystem robotDrive) {
        this.controller = controller;
        this.aprilTagId = aprilTagId;
        this.limelightName = limelightName;
        this.xSpeedLimiter = xSpeedLimiter;
        this.ySpeedLimiter = ySpeedLimiter;
        this.rotLimiter = rotLimiter;
        this.robotDrive = robotDrive;
        addRequirements(robotDrive);
    }

    /**
     * 
     * @param controller    The xbox controller
     * @param aprilTagId    The id for the desired april tag target
     * @param limelightName The name of the limelight assigned on the limelight
     *                      webpage
     * @param xSpeedLimiter The xSpeedLimiter
     * @param ySpeedLimiter The ySpeedLimiter
     * @param rotLimiter    The rotation limiter
     * @param robotDrive    The robot drive subsystem
     */
    public AimAssistCommand(CommandXboxController controller, int aprilTagId, String limelightName,
            SlewRateLimiter xSpeedLimiter, SlewRateLimiter ySpeedLimiter,
            SlewRateLimiter rotLimiter,
            DriveSubsystem robotDrive) {
        this(controller.getHID(), aprilTagId, limelightName, xSpeedLimiter, ySpeedLimiter, rotLimiter,
                robotDrive);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(limelightName, 0);
        LimelightHelpers.setPriorityTagID(limelightName, aprilTagId);
    }

    @Override
    public void execute() {
        boolean hasTarget = LimelightHelpers.getTV(limelightName);

        double xInput = MathUtil.applyDeadband(controller.getLeftX(),
                OIConstants.kDriveDeadband);
        double yInput = MathUtil.applyDeadband(controller.getLeftY(),
                OIConstants.kDriveDeadband);

        double xSpeed = -xSpeedLimiter.calculate(xInput) * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeed = -ySpeedLimiter.calculate(yInput) * DriveConstants.kMaxSpeedMetersPerSecond;

        double tx = LimelightHelpers.getTX(limelightName);
        double currentTagId = LimelightHelpers.getFiducialID(limelightName);

        double rotSpeed = 0.0;

        double desiredArea = 3.0;

        if (hasTarget && currentTagId == aprilTagId) {
            double kP_Rot = AimAssistConstants.kP_Rot;
            rotSpeed = -tx * kP_Rot;
        } else {
            double rotInput = MathUtil.applyDeadband(controller.getRightX(), OIConstants.kDriveDeadband);

            rotSpeed = -rotLimiter.calculate(rotInput) * DriveConstants.kMaxAngularSpeed;
        }

        if (hasTarget && currentTagId == aprilTagId && Math.abs(tx) < desiredArea) {
            controller.setRumble(RumbleType.kBothRumble, 0.5);
        } else {
            controller.setRumble(RumbleType.kBothRumble, 0);
        }

        robotDrive.drive(xSpeed, ySpeed, rotSpeed, true, 0);
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
