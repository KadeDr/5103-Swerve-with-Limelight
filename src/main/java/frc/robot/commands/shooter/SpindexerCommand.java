package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class SpindexerCommand extends Command{
    private final IndexerSubsystem m_indexer;

    public SpindexerCommand(IndexerSubsystem indexer) {
        m_indexer = indexer;

        addRequirements(m_indexer);
    }

    @Override
    public void initialize() {
        m_indexer.spinSerializer();
    }

    @Override
    public void end(boolean interrupted) {
        m_indexer.stopSerializer();
    }
}
