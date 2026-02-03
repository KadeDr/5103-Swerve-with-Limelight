// IntakeCommand.java
package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem m_intake;
    private final double m_targetSpeed;

    // REMOVE SparkFlex from the arguments!
    public IntakeCommand(IntakeSubsystem subsystem, double speed) {
        m_intake = subsystem;
        m_targetSpeed = speed;
        
        // This requirement is now guaranteed to work because we only use the subsystem
        addRequirements(subsystem); 
    }

    @Override
    public void initialize() {
        // Just call the method in the subsystem
        m_intake.runVelocity(m_targetSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopMotor();
    }
}