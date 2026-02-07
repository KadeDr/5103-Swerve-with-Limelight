package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class CalculateTurretPosition extends Command {
    private final ShooterSubsystem m_shooterSubsystem;
    private final String m_limelightName;

    public CalculateTurretPosition(ShooterSubsystem shooterSubsystem, String limelightName) {
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

        double tx = LimelightHelpers.getTX(m_limelightName);

        double turretPosition = m_shooterSubsystem.getPosition();
        double targetPosition = turretPosition + tx;

        double maxRotationLeft = ShooterConstants.kMaxRotationLeft;
        double maxRotationRight = ShooterConstants.kMaxRotationRight;

        if (targetPosition < maxRotationLeft) {
            targetPosition = maxRotationRight;
        } else if (targetPosition > maxRotationRight) {
            targetPosition = maxRotationLeft;
        }

        m_shooterSubsystem.turnTable(targetPosition);
    }
}