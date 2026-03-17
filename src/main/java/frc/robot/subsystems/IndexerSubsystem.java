package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.IndexerConfigs;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    private final SparkFlex m_sparkFlex;
    private final SparkFlex m_secondFlex;
    private final SparkFlex m_serializerFlex;
    private final SparkClosedLoopController m_clc;
    private final SparkClosedLoopController m_serializerCLC;

    public boolean isLocatingTarget = false;

    public IndexerSubsystem(int canId) {
        m_sparkFlex = new SparkFlex(canId, MotorType.kBrushless);
        m_secondFlex = new SparkFlex(IndexerConstants.secondCanId, MotorType.kBrushless);
        m_serializerFlex = new SparkFlex(IndexerConstants.spindexerCanId, MotorType.kBrushless);
        m_clc = m_sparkFlex.getClosedLoopController();
        m_serializerCLC = m_serializerFlex.getClosedLoopController();
        m_sparkFlex.configure(IndexerConfigs.mainConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_secondFlex.configure(IndexerConfigs.followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_serializerFlex.configure(IndexerConfigs.serializerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Indexer Speed", getRPM());
    }

    public void spin(double rpm) {
        m_clc.setSetpoint(rpm, ControlType.kVelocity);
        m_serializerCLC.setSetpoint(-800, ControlType.kVelocity);
    }

    public void spinSerializer() {
        m_serializerCLC.setSetpoint(-1000, ControlType.kVelocity);
    }

    public void stopMotor() {
        m_sparkFlex.set(0);
        m_serializerFlex.set(0);
    }

    public void stopSerializer() {
        m_serializerFlex.set(0);
    }

    public double getRPM() {
        return m_sparkFlex.getEncoder().getVelocity();
    }
}
