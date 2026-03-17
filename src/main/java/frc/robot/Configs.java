package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
        public static final class ShooterConfigs {
                public static final SparkMaxConfig shooterConfig = new SparkMaxConfig();
                public static final SparkMaxConfig positionTurntableConfig = new SparkMaxConfig();
                public static final SparkMaxConfig velocityTurntableConfig = new SparkMaxConfig();

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

                        positionTurntableConfig.encoder
                                        .positionConversionFactor(9)
                                        .velocityConversionFactor(0.025);
                        positionTurntableConfig.closedLoop
                                        .p(0.025) // 0.038 worked for position, .0015 for velocity control
                                        .outputRange(-.4, .4);

                        velocityTurntableConfig.encoder
                                        .positionConversionFactor(9)
                                        .velocityConversionFactor(0.025);
                        velocityTurntableConfig.closedLoop
                                        .p(0.0015)
                                        .outputRange(-.1, .1);
                }
        }

        public static final class IndexerConfigs {
                public static final SparkFlexConfig mainConfig = new SparkFlexConfig();
                public static final SparkFlexConfig followerConfig = new SparkFlexConfig();
                public static final SparkFlexConfig serializerConfig = new SparkFlexConfig();

                static {
                        mainConfig
                                        .smartCurrentLimit(60);
                        mainConfig.softLimit
                                        .forwardSoftLimitEnabled(false)
                                        .reverseSoftLimitEnabled(false);
                        mainConfig.encoder
                                        .positionConversionFactor(0.25)
                                        .velocityConversionFactor(0.25);
                        mainConfig.closedLoop
                                        .outputRange(-1, 1)
                                        .p(0.0001).feedForward.kV(0.0002);

                        followerConfig
                                        .apply(mainConfig)
                                        .follow(IndexerConstants.canId, true);

                        serializerConfig
                                        .smartCurrentLimit(60);
                        serializerConfig.softLimit
                                        .forwardSoftLimitEnabled(false)
                                        .reverseSoftLimitEnabled(false);
                        serializerConfig.encoder
                                        .positionConversionFactor(0.25) // Assuming the serializer uses the same ratio
                                        .velocityConversionFactor(0.25);
                        serializerConfig.closedLoop
                                        .outputRange(-1, 1)
                                        .p(0.0003).feedForward.kV(0.0003);
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
                                        .pid(0.04, 0, 0)
                                        .outputRange(-1, 1).feedForward.kV(drivingVelocityFeedForward);

                        turningConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(20);
                        turningConfig.absoluteEncoder
                                        .inverted(true)
                                        .positionConversionFactor(turningFactor) // radians
                                        .velocityConversionFactor(turningFactor / 60.0); // radians per second
                        turningConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                        .pid(1, 0, 0)
                                        .outputRange(-1, 1)
                                        .positionWrappingEnabled(true)
                                        .positionWrappingInputRange(0, turningFactor);
                }
        }
}
