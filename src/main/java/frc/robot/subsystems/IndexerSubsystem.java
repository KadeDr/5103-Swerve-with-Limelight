package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IndexerConfigs;

public class IndexerSubsystem extends SubsystemBase {
    private final SparkMax m_sparkMax;
    private final SparkClosedLoopController m_clc;

    public IndexerSubsystem(int canId) {
        m_sparkMax = new SparkMax(canId, MotorType.kBrushless);
        m_clc = m_sparkMax.getClosedLoopController();
        m_sparkMax.configure(IndexerConfigs.mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Indexer Speed", getRPM());
    }

    public void spin(double rpm) {
        m_clc.setSetpoint(rpm, ControlType.kVelocity);
    }

    public void stopMotor() {
        m_sparkMax.set(0);
    }

    public double getRPM() {
        return m_sparkMax.getEncoder().getVelocity();
    }
}
