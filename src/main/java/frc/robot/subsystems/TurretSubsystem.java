package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ShooterConfigs;

public class TurretSubsystem extends SubsystemBase{
    private final SparkMax m_turntable;
    private final SparkClosedLoopController m_turntableCLC;
    private RelativeEncoder m_turnTableEncoder;

    public TurretSubsystem(int turnTableCanId) {
        m_turntable = new SparkMax(turnTableCanId, MotorType.kBrushless);
        m_turntableCLC = m_turntable.getClosedLoopController();
        m_turnTableEncoder = m_turntable.getEncoder();
        m_turntable.configure(ShooterConfigs.positionTurntableConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void reconfigureTurntable(SparkMaxConfig config) {
        m_turntable.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void turnTable(double targetPosition) {
        m_turntableCLC.setSetpoint(targetPosition, ControlType.kPosition);
    }

    public void stopMotors() {
        m_turntable.set(0);
    }

    public void turnTableVelocity(double targetSpeed) {
        m_turntableCLC.setSetpoint(targetSpeed, ControlType.kVelocity);
    }

    public double getPosition() {
        return m_turnTableEncoder.getPosition();
    }

    public void ResetEncoder() {
        m_turnTableEncoder.setPosition(0);
    }
}
