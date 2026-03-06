package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
        public static final class ShooterConfigs {
                public static final SparkMaxConfig shooterConfig = new SparkMaxConfig();
                public static final SparkMaxConfig turntableConfig = new SparkMaxConfig();

                static {
                        shooterConfig
                                        .idleMode(IdleMode.kCoast)
                                        .smartCurrentLimit(80);
                        shooterConfig.softLimit
                                        .forwardSoftLimitEnabled(false)
                                        .reverseSoftLimitEnabled(false);
                        shooterConfig.closedLoop
                                        .p(0.00022).feedForward.kV(0.00017);
                        // .feedForward.kV(0.0000001);

                        turntableConfig.encoder
                                        .positionConversionFactor(9)
                                        .velocityConversionFactor(0.025);
                        turntableConfig.closedLoop
                                        .p(0.025) // 0.038 worked for position, .0015 for velocity control
                                        .outputRange(-.4, .4);
                }
        }

        public static final class IndexerConfigs {
                public static final SparkMaxConfig mainConfig = new SparkMaxConfig();

                static {
                        mainConfig
                                        .smartCurrentLimit(60);
                        mainConfig.softLimit
                                        .forwardSoftLimitEnabled(false)
                                        .reverseSoftLimitEnabled(false);
                        mainConfig.closedLoop
                                        .outputRange(-1, 1)
                                        .p(0.005103)
                                        .feedForward.kV(0.000285);
                        // .feedForward.kV(0.0000001);
                }
        }

        public static final class IntakeConfigs {
                public static final SparkFlexConfig mainConfig = new SparkFlexConfig();
                public static final SparkFlexConfig invertedConfig = new SparkFlexConfig();

                static {
                        mainConfig
                                        .smartCurrentLimit(60);
                        mainConfig.softLimit
                                        .forwardSoftLimitEnabled(false)
                                        .reverseSoftLimitEnabled(false);
                        mainConfig.closedLoop
                                        .outputRange(-1, 1)
                                        .p(0.000118)
                                                        // .i(0.00004)
                                                        // .iZone(50)
                                                        .feedForward
                                        .kV(0.00018);
                        // .feedForward.kV(0.0000001);

                        invertedConfig
                                        .follow(IntakeConstants.canId, true);
                }
        }

        public static final class MAXSwerveModule {
                public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
                public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

                static {
                        // Use module constants to calculate conversion factors and feed forward gain.
                        double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                                        / ModuleConstants.kDrivingMotorReduction;
                        double turningFactor = 2 * Math.PI;
                        double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

                        drivingConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(40);
                        drivingConfig.encoder
                                        .positionConversionFactor(drivingFactor) // meters
                                        .velocityConversionFactor(drivingFactor / 60.0); // meters per second
                        drivingConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        // These are example gains you may need to them for your own robot!
                                        .pid(0.04, 0, 0)
                                        .outputRange(-1, 1).feedForward.kV(drivingVelocityFeedForward);

                        turningConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(20);
                        turningConfig.absoluteEncoder
                                        // Invert the turning encoder, since the output shaft rotates in the opposite
                                        // direction of the steering motor in the MAXSwerve Module.
                                        .inverted(true)
                                        .positionConversionFactor(turningFactor) // radians
                                        .velocityConversionFactor(turningFactor / 60.0); // radians per second
                        turningConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                        // These are example gains you may need to them for your own robot!
                                        .pid(1, 0, 0)
                                        .outputRange(-1, 1)
                                        // Enable PID wrap around for the turning motor. This will allow the PID
                                        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                                        // to 10 degrees will go through 0 rather than the other direction which is a
                                        // longer route.
                                        .positionWrappingEnabled(true)
                                        .positionWrappingInputRange(0, turningFactor);
                }
        }
}
