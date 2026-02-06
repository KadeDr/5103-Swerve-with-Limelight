package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class LocateAprilTagCommand extends Command {
    private final ShooterSubsystem m_shooterSubsystem;
    private final String m_limelightName;

    public LocateAprilTagCommand(ShooterSubsystem shooterSubsystem, String limelightName) {
        m_shooterSubsystem = shooterSubsystem;
        m_limelightName = limelightName;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(m_limelightName, 0);
        LimelightHelpers.setPriorityTagID(m_limelightName, 0);
    }

    @Override
    public void execute() {
        boolean tv = LimelightHelpers.getTV(m_limelightName);

        if (!tv) {
            return;
        }

        double turretPosition = m_shooterSubsystem.getPosition();

        double maxRotationLeft = ShooterConstants.kMaxRotationLeft;
        double maxRotationRight = ShooterConstants.kMaxRotationRight;

        double targetPosition = maxRotationLeft;

        if (turretPosition <= maxRotationLeft) {
            targetPosition = maxRotationRight;
        } else if (turretPosition >= maxRotationRight) {
            targetPosition = maxRotationLeft;
        }

        m_shooterSubsystem.turnTable(targetPosition);
    }
}