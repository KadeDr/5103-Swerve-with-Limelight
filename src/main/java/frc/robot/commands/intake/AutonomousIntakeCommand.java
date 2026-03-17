package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AutonomousIntakeCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final double m_targetSpeed;

    public AutonomousIntakeCommand(IntakeSubsystem subsystem, double speed) {
        m_intake = subsystem;
        m_targetSpeed = speed;
        
        addRequirements(subsystem); 
    }

    @Override
    public void execute() {
        m_intake.runVelocity(m_targetSpeed);
        Timer.delay(1);
        m_intake.runVelocity(-m_targetSpeed);
        Timer.delay(0.125);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopMotor();
    }
}