package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class SpinIndexerCommand extends Command {
    private final double m_speed;
    private final IndexerSubsystem m_subsystem;

    public SpinIndexerCommand(IndexerSubsystem subsystem, double speed) {
        m_speed = speed;
        m_subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.spin(m_speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopMotor();
    }
}
