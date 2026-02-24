package frc.robot.climb.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.demacia.utils.motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.sensors.DigitalEncoderConfig;
import frc.demacia.utils.motors.TalonFXConfig;
import frc.demacia.utils.motors.TalonSRXConfig;

public class ClimbConstants {
        public static final Canbus CANBUS_CLIMB = Canbus.Rio;

        // arm constans
        public static final boolean WITH_BRAKE_ARMS = true;
        public static final double MAX_CURRENT = 20;
        public static final int MOTOR_ID_ARMS = 61;
        public static final double DIAMETER_ARMS = 0;
        public static final double GEAR_RATIO_ARMS = 24;
        public static final boolean WITH_INVERT_ARMS = false;
        public static final double ANGLE_ARMS_RAISED = 0.3232698840543895; // radians
        public static final double powerToRaiseArmsAfterClimb = 0.2;
        public static final double ANGLE_ARMS_LOWERED = Math.toRadians(22.86102);
        public static final double ARMS_ANGLE_CLOSED = Math.toRadians(160);
        public static final double ARMS_OFFSET = 1.6465087097464108;

        public static final double ARMS_KP = 3;

        // climb arms motor
        public static final TalonSRXConfig ARMS_MOTOR_CONFIG = new TalonSRXConfig(MOTOR_ID_ARMS, "arms motor")
                        .withBrake(WITH_BRAKE_ARMS)
                        .withCurrent(MAX_CURRENT)
                        .withInvert(WITH_INVERT_ARMS);

        // CLIMB LEVER MOTOR PID AND FEEDFORWARD
        public static final double LEVER_KS = 0;
        public static final double LEVER_KV = 0;
        public static final double LEVER_KA = 0;
        public static final double LEVER_KP = 4.5;
        public static final double LEVER_KG = 0;
        public static final double LEVER_KD = 0;
        public static final double LEVER_KI = 0;

        // lever constans
        public static final int MOTOR_ID_LEVER = 60;
        public static final boolean WITH_BRAKE_LEVER = true;
        public static final double LEVER_GEAR_RATIO = 100;
        public static final boolean WITH_INVERT_LEVER = false;
        public static final double ANGLE_LEVER_CLOSED = 0; // radians
        public static final double ANGLE_LEVER_OPEN = 1.66; // radians
        public static final double ANGLE_LEVER_MID = Math.toRadians(40);
        public static final double powerOpen = 0.8;
        public static final double powerMid = 0.2;

        // lever climb motor config
        public static final TalonFXConfig LEVER_MOTOR_CONFIG = new TalonFXConfig(MOTOR_ID_LEVER, CANBUS_CLIMB,
                        "motor lever")
                        .withPID(LEVER_KP, LEVER_KI, LEVER_KD, LEVER_KS, LEVER_KV, LEVER_KA, LEVER_KG)
                        .withBrake(WITH_BRAKE_LEVER)
                        .withCurrent(MAX_CURRENT)
                        .withInvert(WITH_INVERT_LEVER)
                        .withRadiansMotor(LEVER_GEAR_RATIO);

        // the climb arm encoder
        public static final DigitalEncoderConfig DIGITAL_ENCODER_CONFIG = new DigitalEncoderConfig(3, "climb encoder")
                        .withInvert(true);

        // constans to auto drive to tower
        public static final double velocityToGoBackAfterClimb = 0.3;
        public static final double timeToGoBackAfterClimb = 1;
        public static final double rotationKp = 2.2;
        public static final double driveKp = 1.3;
        public static final double velocityToStraightenArms = 0.5;
        public static final double velocityToRaiseArmsAfterClimb = 0.5;
        public static final double CHASSIS_TOLERANCE = Math.toRadians(5); // radians
        public static final double timeToStraightenArms = 0.2; // seconds
        public static final double timeToRaiseArmsAfterClimb = 0.1; // seconds

        // collision / safety thresholds
        // Current (amps) above which we consider the motor to be stalled/colliding
        public static final double ARM_COLLISION_CURRENT_THRESHOLD = 15.0;
        // How long to run the recovery (reverse) action after detecting a collision (seconds)
        public static final double ARM_COLLISION_RECOVER_TIME = 0.5;
        // Power to apply during recovery (positive should lower the arms)
        public static final double ARM_COLLISION_RECOVER_POWER = 0.2;

        // Lever collision threshold (amps)
        public static final double LEVER_COLLISION_CURRENT_THRESHOLD = 15.0;

        // climb pose
        public static Pose2d targetRightSideRed = new Pose2d(15.439, 3.863, new Rotation2d());
        public static Pose2d targetLeftSideRed = new Pose2d(15.43, 4.720, new Rotation2d());
        public static Pose2d targetLeftSideBlue = new Pose2d(1, 1, new Rotation2d(180));
        public static Pose2d targetRightSideBlue = new Pose2d(1, 1, new Rotation2d(180));


        public static class Calibration {
                public static final double POWER = 0.3;
                public static final double CURRENT_FOR_CALIBRATE = 12; //need to test
                public static final double TIME_TO_CONFIRM_CALIBRATION = 0.15; // seconds
                public static final double POSITION_AFTER_CALIBRATION = Math.toRadians(110); //need to check
                
        }

}
