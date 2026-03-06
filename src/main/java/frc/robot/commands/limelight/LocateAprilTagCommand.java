package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class LocateAprilTagCommand extends Command {
    private final ShootSubsystem m_shootSubsystem;
    private final TurretSubsystem m_turretSubsystem;
    private final String m_limelightName;

    public LocateAprilTagCommand(ShootSubsystem shootSubsystem, TurretSubsystem turretSubsystem, String limelightName) {
        m_shootSubsystem = shootSubsystem;
        m_turretSubsystem = turretSubsystem;
        m_limelightName = limelightName;
        addRequirements(m_shootSubsystem);
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

        double turretPosition = m_turretSubsystem.getPosition();

        double maxRotationLeft = ShooterConstants.kMaxRotationLeft;
        double maxRotationRight = ShooterConstants.kMaxRotationRight;

        double targetPosition = maxRotationLeft;

        if (turretPosition <= maxRotationLeft) {
            targetPosition = maxRotationRight;
        } else if (turretPosition >= maxRotationRight) {
            targetPosition = maxRotationLeft;
        }

        m_turretSubsystem.turnTable(targetPosition);
    }
}