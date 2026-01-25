// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

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
        static { // distance (M) Velocity (M/s) (velocity of the flywheel, not the ball) Hood
                 // angle (Radians)

                SHOOTER_LOOKUP_TABLE.add(1.5, 14, Math.toRadians(85));
                SHOOTER_LOOKUP_TABLE.add(1.96, 14.5, Math.toRadians(78));
                SHOOTER_LOOKUP_TABLE.add(3, 16.7, Math.toRadians(71));
                SHOOTER_LOOKUP_TABLE.add(4, 18.6, Math.toRadians(68));
                SHOOTER_LOOKUP_TABLE.add(4.79, 19, Math.toRadians(62));
                // SHOOTER_LOOKUP_TABLE.add(1, 0, 0);
                // SHOOTER_LOOKUP_TABLE.add(1.5, 0, 0);
                // SHOOTER_LOOKUP_TABLE.add(2, 0, 0);
                // SHOOTER_LOOKUP_TABLE.add(2.5, 0, 0);
                // SHOOTER_LOOKUP_TABLE.add(3, 0, 0);
                // SHOOTER_LOOKUP_TABLE.add(3.5, 0, 0);
                // SHOOTER_LOOKUP_TABLE.add(4, 0, 0);
                // SHOOTER_LOOKUP_TABLE.add(4.5, 0, 0);
                // SHOOTER_LOOKUP_TABLE.add(5, 0, 0);
        }

        public static final int shooterMotorID = 30;
        public static final Canbus shooterMotorCanbus = Canbus.Rio;
        public static final String shooterMotorName = "Shooter Motor";

        public static final TalonFXConfig SHOOTER_MOTOR_CONFIG = new TalonFXConfig(shooterMotorID, shooterMotorCanbus,
                        shooterMotorName)
                        .withFeedForward(0.004, 0)
                        .withInvert(true)
                        .withRampTime(0.3)
                        .withBrake(false)
                        .withPID(2.2, 0, 0, 0.25073, 0.27, 0, 0)
                        .withMeterMotor(1, 4 * 0.0254);

        public static final int INDEXER_ID = 32;
        public static final Canbus CANBUS_MOVE_TO_SOTER_MOTOR = Canbus.Rio;
        public static final String INDEXER_NAME = "Indexer Motor";

        public static final TalonFXConfig INDEXER_CONFIG = new TalonFXConfig(INDEXER_ID, CANBUS_MOVE_TO_SOTER_MOTOR,
                        INDEXER_NAME)
                        .withInvert(true);

        public static final int HOOD_ID = 31;
        public static final String HOOD_NAME = "HOOD MOTOR";

        public static final double HOOD_KP = 30;
        public static final double HOOD_KI = 0.5;
        public static final double HOOD_KD = 0;
        public static final double HOOD_KS = 0;
        public static final double HOOD_KV = 0;
        public static final double HOOD_KA = 0;
        public static final double HOOD_KG = 0;
        public static final double HOOD_GEAR_RATIO = ((110 / 25d) * 32);

        public static final TalonFXConfig HOOD_CONFIG = new TalonFXConfig(HOOD_ID, shooterMotorCanbus, HOOD_NAME)
                        .withBrake(true)
                        .withRadiansMotor(HOOD_GEAR_RATIO)
                        .withInvert(false)
                        .withPID(HOOD_KP, HOOD_KI, HOOD_KD, HOOD_KS, HOOD_KV, HOOD_KV, HOOD_KG)
                        .withFeedForward(HOOD_KV, HOOD_GEAR_RATIO);

        public static final double MAX_ANGLE_HOOD = Math.toRadians(85d);
        public static final double MIN_ANGLE_HOOD = Math.toRadians(45d);

        public static final LimitSwitchConfig LIMIT_SWITCH_CONFIG = new LimitSwitchConfig(0, "Limit Switch");

        public static final Translation3d HUB_POSE_Translation3d = new Translation3d(11.265 + 0.5969, 4.023, 1.829);
        public static final Translation2d HUB_POSE_Translation2d = HUB_POSE_Translation3d.toTranslation2d();
        public static final Pose2d hubPose2d = new Pose2d(HUB_POSE_Translation2d, new Rotation2d());
}
