package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final double m_targetSpeed;

    public IntakeCommand(IntakeSubsystem subsystem, double speed) {
        m_intake = subsystem;
        m_targetSpeed = speed;
        
        addRequirements(subsystem); 
    }

    @Override
    public void initialize() {
        m_intake.runVelocity(m_targetSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopMotor();
    }
}