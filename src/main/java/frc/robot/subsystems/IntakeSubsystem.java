package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IntakeConfigs;

public class IntakeSubsystem extends SubsystemBase{
    private SparkFlex m_sparkFlex;
    private final SparkClosedLoopController m_clc;

    public IntakeSubsystem(int canId) {
        m_sparkFlex = new SparkFlex(canId, MotorType.kBrushless);
        m_clc = m_sparkFlex.getClosedLoopController();
        m_sparkFlex.configure(IntakeConfigs.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runVelocity(double targetRpm) {
        // We don't need the extra '0.000147' here because we set it in the config above!
        m_clc.setSetpoint(targetRpm, ControlType.kVelocity);
    }

    public void stopMotor() {
        m_sparkFlex.set(0);
    }
}
