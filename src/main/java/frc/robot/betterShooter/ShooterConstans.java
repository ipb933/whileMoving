// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.betterShooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.demacia.utils.LookUpTable;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.sensors.LimitSwitchConfig;
import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */

public class ShooterConstans {

    public static final LookUpTable SHOOTER_LOOKUP_TABLE = new LookUpTable(2);
    static {SHOOTER_LOOKUP_TABLE.add(1.5, 13.7, Math.toRadians(85));
            SHOOTER_LOOKUP_TABLE.add(1.8, 14, Math.toRadians(83));
            SHOOTER_LOOKUP_TABLE.add(3, 15.5, Math.toRadians(70));
            SHOOTER_LOOKUP_TABLE.add(3.4, 16.3, Math.toRadians(65));
            SHOOTER_LOOKUP_TABLE.add(4.2, 18, Math.toRadians(58));}

    public class FlyWheelConstans {
        public static final int FLYWHEEL_ID = 30;
        public static final Canbus FLYWHEEL_CABUS = Canbus.Rio;
        public static final String FLYWHEEL_NAME = "Flywheel Motor";
        
        public static final double FLYWHEEL_KP = 3.5;
        public static final double FLYWHEEL_KI = 0;
        public static final double FLYWHEEL_KD = 0.04;
        public static final double FLYWHEEL_KS = 0.25073;
        public static final double FLYWHEEL_KV = 0.29;
        public static final double FLYWHEEL_KA = 0;
        public static final double FLYWHEEL_KG = 0;
        public static final double FLYWHEEL_KSIN = 0;
        public static final double FLYWHEEL_KV2 = 0.0031;
        
        public static final double FLYWHEEL_GEAR_RATIO = 1;
        public static final double FLYWHEEL_DIAMETER = 4 * 0.0254;

        public static final TalonFXConfig FLYWHEEL_CONFIG = new TalonFXConfig(FLYWHEEL_ID, FLYWHEEL_CABUS, FLYWHEEL_NAME)
            .withInvert(true)
            .withBrake(false)
            .withPID(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD, FLYWHEEL_KS, FLYWHEEL_KV, FLYWHEEL_KA, FLYWHEEL_KG)
            .withFeedForward(FLYWHEEL_KV2, FLYWHEEL_KSIN)
            .withMeterMotor(FLYWHEEL_GEAR_RATIO, FLYWHEEL_DIAMETER);
    }
    
    public class HoodConstans {
        public static final int HOOD_ID = 31;
        public static final Canbus HOOD_CANBUS = Canbus.Rio;
        public static final String HOOD_NAME = "HOOD MOTOR";

        public static final double HOOD_KP = 1.2;
        public static final double HOOD_KI = 0;
        public static final double HOOD_KD = 0;
        public static final double HOOD_KS = 0.083;
        public static final double HOOD_KV = 2;
        public static final double HOOD_KA = 0.3;
        public static final double HOOD_KG = 0;
        public static final double HOOD_MAX_VELOCITY = Math.PI;
        public static final double HOOD_MAX_ACCEL = 2 * Math.PI;
        public static final double HOOD_MAX_JERK = 10 * Math.PI;
        public static final double HOOD_GEAR_RATIO = ((110 / 25d) * 32);

        public static final TalonFXConfig HOOD_CONFIG = new TalonFXConfig(HOOD_ID, HOOD_CANBUS, HOOD_NAME)
            .withBrake(true)
            .withRadiansMotor(HOOD_GEAR_RATIO)
            .withInvert(false)
            .withPID(HOOD_KP, HOOD_KI, HOOD_KD, HOOD_KS, HOOD_KV, HOOD_KA, HOOD_KG)
            .withMotionParam(HOOD_MAX_VELOCITY, HOOD_MAX_ACCEL, HOOD_MAX_JERK);

        public static final double MAX_ANGLE_HOOD = Math.toRadians(85d);
        public static final double MIN_ANGLE_HOOD = Math.toRadians(45d);

        
        public static final String LIMIT_SWITCH_NAME = "Limit Switch";

        public static final LimitSwitchConfig LIMIT_SWITCH_CONFIG = new LimitSwitchConfig(0, LIMIT_SWITCH_NAME);
    }

    public class IndexerConstans {
        public static final int INDEXER_ID = 32;
        public static final Canbus CANBUS_MOVE_TO_SOTER_MOTOR = Canbus.Rio;
        public static final String INDEXER_NAME = "Indexer Motor";
    
        public static final TalonFXConfig INDEXER_CONFIG = new TalonFXConfig(INDEXER_ID, CANBUS_MOVE_TO_SOTER_MOTOR,
            INDEXER_NAME)
            .withInvert(true);
    }

    

    public static final Translation3d HUB_POSE_Translation3d = new Translation3d(11.265 + 0.5969, 4.023, 1.829);
    public static final Translation2d HUB_POSE_Translation2d = HUB_POSE_Translation3d.toTranslation2d();
    public static final Pose2d hubPose2d = new Pose2d(HUB_POSE_Translation2d, new Rotation2d());
}