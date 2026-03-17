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
    private final SparkFlex m_sparkFlex;
    private final SparkFlex m_leftFlex;
    private final SparkClosedLoopController m_clc;

    public IntakeSubsystem(int canId, int leftCanId) {
        m_sparkFlex = new SparkFlex(canId, MotorType.kBrushless);
        m_leftFlex = new SparkFlex(leftCanId, MotorType.kBrushless);
        m_clc = m_sparkFlex.getClosedLoopController();
        m_sparkFlex.configure(IntakeConfigs.mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_leftFlex.configure(IntakeConfigs.invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runVelocity(double targetRpm) {
        m_clc.setSetpoint(targetRpm, ControlType.kVelocity);
    }

    public void Test(double speed) {
        m_sparkFlex.set(speed);
    }

    public void stopMotor() {
        m_sparkFlex.set(0);
    }
}
