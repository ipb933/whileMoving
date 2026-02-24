package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotCommon {
    public enum Shifts {
        Auto, Transition, Active, Inactive, Endgame
    }

    public enum RobotStates {
        HubWithAutoIntake, HubWithoutAutoIntake, DeliveryWithAutoIntake, DeliveryWithoutAutoIntake, 
        DriveAutoIntake, Drive, Trench, PrepareClimb, Climb, GetOffClimb, Test, IDLE;
    }

    public static RobotStates currentState = RobotStates.Drive;
    public static Pose2d currentRobotPose = Pose2d.kZero;
    public static Pose2d futureRobotPose = Pose2d.kZero; // 0.04 seconds in advance
    public static ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
    public static ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
    public static Rotation2d robotAngle = Rotation2d.kZero;
    public static Translation2d deliveryTarget = Translation2d.kZero;
    public static double targetAccuracy = 1; // 0-1
    public static double currentDistanceFromTarget = 0;
    public static double currentAngleFromTarget = 0;
    public static double futureDistanceFromTarget = 0;

    public static double futureAngleFromTarget = 0;
    public static double futureAngleFromTargetRobotRelative = 0;
    public static double currentWantedTurretAngle = 0;
    public static double futureWantedTurretAngle = 0;
    public static double currentTurrentAngle = 0;
    public static boolean isRed = false;
    public static boolean isComp = false;
    public static boolean isRobotCalibrated = false;
    public static Shifts currentShift = Shifts.Auto;

    public static Command changeState(RobotStates newState) {
        RobotContainer.leds.changeColor(newState);
        return new InstantCommand(() -> currentState = newState).ignoringDisable(true);
    }
}