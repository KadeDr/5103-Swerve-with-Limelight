package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Configs.ShooterConfigs;

public class ShootSubsystem extends SubsystemBase{
    private SparkMax m_shooter;
    private SparkClosedLoopController m_shooterCLC;
    private RelativeEncoder m_shooterEncoder;

    public ShootSubsystem(int shooterCanId) {
        m_shooter = new SparkMax(shooterCanId, MotorType.kBrushless);
        m_shooterCLC = m_shooter.getClosedLoopController();
        m_shooterEncoder = m_shooter.getEncoder();
        m_shooter.configure(ShooterConfigs.shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runVelocity(double targetRPM) {
        m_shooterCLC.setSetpoint(targetRPM, ControlType.kVelocity);
    }

    public void stopMotor() {
        m_shooter.set(0);
    }

    public double getRPM() {
        return m_shooterEncoder.getVelocity(); 
    }
}
