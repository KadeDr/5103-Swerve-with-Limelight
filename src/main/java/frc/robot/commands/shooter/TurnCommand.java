package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TurnCommand extends Command {
    private final ShooterSubsystem m_shooterSubsystem;
    private final double m_targetPosition;

    public TurnCommand(ShooterSubsystem shooterSubsystem, double targetPosition) {
        m_shooterSubsystem = shooterSubsystem;
        m_targetPosition = targetPosition;
        addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {
        m_shooterSubsystem.turnTable(m_targetPosition);
    }

    @Override
    public void end (boolean interrupted) {
        m_shooterSubsystem.stopTurnTable();
    }
}
