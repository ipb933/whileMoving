package frc.robot.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.chassis.ChassisConfig;
import frc.demacia.utils.chassis.SwerveModuleConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.sensors.CancoderConfig;
import frc.demacia.utils.sensors.PigeonConfig;
import frc.demacia.vision.subsystem.Tag;

public class ChassisConstants {
    public static final String NAME = "chassis";

    public static final int PIGEON_ID = 14;
    public static final Canbus CAN_BUS = Canbus.Rio;
    public static final Canbus PIGEON_CAN_BUS = Canbus.Rio;
    public static final double STEER_GEAR_RATIO = 287.0/11.0;
    public static final double DRIVE_GEAR_RATIO = 6.03;

    public static final double STEER_KP = 4.1;
    public static final double STEER_KI = 0;
    public static final double STEER_KD = 0;
    public static final double STEER_KS = 0.19817640545050964;
    public static final double STEER_KV = 0.3866402641515461;
    public static final double STEER_KA = 0.05;

    public static final double DRIVE_KP = 19;
    public static final double DRIVE_KI = 0;
    public static final double DRIVE_KD = 0;
    public static final double DRIVE_KS = 0.14677232883614777;
    public static final double DRIVE_KV = 2.947;
    public static final double DRIVE_KA = 0.08058;

    public static final double MOTION_MAGIC_VEL = 15 * 2 * Math.PI;
    public static final double MOTION_MAGIC_ACCEL = 8 * 2 * Math.PI;
    public static final double MOTION_MAGIC_JERK = 160 * 2 * Math.PI;

    public static final double RAMP_TIME_STEER = 0.25;

    public static class SwerveModuleConfigs {
        public final TalonFXConfig STEER_CONFIG;
        public final TalonFXConfig DRIVE_CONFIG;
        public final CancoderConfig CANCODER_CONFIG;
        public final Translation2d POSITION;
        public final double STEER_OFFSET;
        public final String NAME;
        public SwerveModuleConfig SWERVE_MODULE_CONFIG;

        public SwerveModuleConfigs(TalonFXConfig steerConfig, TalonFXConfig driveConfig, CancoderConfig cancoderConfig,
                Translation2d position, double steerOffset, String name) {
            STEER_CONFIG = steerConfig;
            DRIVE_CONFIG = driveConfig;
            CANCODER_CONFIG = cancoderConfig;
            POSITION = position;
            STEER_OFFSET = steerOffset;
            NAME = name;
        }

        public SwerveModuleConfigs(int swerveId, double steerOffset, double wheelDiameter) {
            switch (swerveId) {
                case 0:
                    NAME = "Front Left";
                    break;
                case 1:
                    NAME = "Front Right";
                    break;
                case 2:
                    NAME = "Back Left";
                    break;
                case 3:
                    NAME = "Back Right";
                    break;

                default:
                    NAME = "";
                    break;
            }
            STEER_CONFIG = new TalonFXConfig(swerveId * 3 + 2, CAN_BUS, NAME + " Steer")
                    .withPID(STEER_KP, STEER_KI, STEER_KD, STEER_KS, STEER_KV, STEER_KA, 0)
                    .withMotionParam(MOTION_MAGIC_VEL, MOTION_MAGIC_ACCEL, MOTION_MAGIC_JERK)
                    .withBrake(true)
                    .withRadiansMotor(STEER_GEAR_RATIO)
                    .withRampTime(RAMP_TIME_STEER);
            DRIVE_CONFIG = new TalonFXConfig(swerveId * 3 + 1, CAN_BUS, NAME + " Drive")
                    .withPID(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KS, DRIVE_KV, DRIVE_KA, 0)
                    .withBrake(true)
                    .withMeterMotor(DRIVE_GEAR_RATIO ,wheelDiameter);
            CANCODER_CONFIG = new CancoderConfig(swerveId * 3 + 3, CAN_BUS, NAME + " Cancoder");
            POSITION = new Translation2d(
                    swerveId == 0 || swerveId == 1 ? 0.34 : -0.34,
                    swerveId == 0 || swerveId == 2 ? 0.29 : -0.29);
            STEER_OFFSET = steerOffset;
            SWERVE_MODULE_CONFIG = new SwerveModuleConfig(NAME, STEER_CONFIG, DRIVE_CONFIG, CANCODER_CONFIG)
                .withSteerOffset(steerOffset);
        }
    }

    public static final SwerveModuleConfigs FRONT_LEFT = new SwerveModuleConfigs(
            0,
            2.8608725004356236738465754211232,
            0.1);

    public static final SwerveModuleConfigs FRONT_RIGHT = new SwerveModuleConfigs(
            1,
            -2.9636528456904673494361192620506,
            0.1);

    public static final SwerveModuleConfigs BACK_LEFT = new SwerveModuleConfigs(
            2,
            -1.5401155329399386984644955163586,
            0.1);

    public static final SwerveModuleConfigs BACK_RIGHT = new SwerveModuleConfigs(
            3,
            1.4756877508001192187301036258543,
            0.1);

    public static final PigeonConfig PIGEON_CONFIG = new PigeonConfig(PIGEON_ID, PIGEON_CAN_BUS, NAME + " pigeon");

    public static final Translation2d FRONT_LEFT_POSITION = new Translation2d(0.34, 0.29);
    public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(0.34, -0.29);
    public static final Translation2d BACK_LEFT_POSITION = new Translation2d(-0.34, 0.29);
    public static final Translation2d BACK_RIGHT_POSITION = new Translation2d(-0.34, -0.29);

    public static final ChassisConfig CHASSIS_CONFIG = new ChassisConfig(
        NAME,
        FRONT_LEFT.SWERVE_MODULE_CONFIG,
        FRONT_RIGHT.SWERVE_MODULE_CONFIG,
        BACK_LEFT.SWERVE_MODULE_CONFIG,
        BACK_RIGHT.SWERVE_MODULE_CONFIG,
        PIGEON_CONFIG,
        FRONT_LEFT_POSITION,
        FRONT_RIGHT_POSITION,
        BACK_LEFT_POSITION,
        BACK_RIGHT_POSITION,
        new Tag[] {});
}