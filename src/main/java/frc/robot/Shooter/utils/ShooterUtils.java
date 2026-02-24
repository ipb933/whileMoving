
package frc.robot.shooter.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.shooter.subsystem.Shooter;

public class ShooterUtils {
  /** Creates a new shooterUtilse. */

  static Shooter shooter;

  public ShooterUtils() {

  }

  // public static Translation3d GetChassisVelAsVector() {
  // return new
  // Translation3d(RobotContainer.chassis.getRobotRelVelocities().vxMetersPerSecond,
  // RobotContainer.chassis.getRobotRelVelocities().vyMetersPerSecond, 0);
  // }

  public static double distensFromToPose2dPoint(Pose2d from, Pose2d to) {
    return from.getTranslation().getDistance(to.getTranslation());
  }

  public static double angle_betuenTowPose2d(Pose2d from, Pose2d to) {
    return to.getTranslation().minus(from.getTranslation()).getAngle().getRadians();
  }

  public static Pose2d computeFuturePosition(ChassisSpeeds speeds, Pose2d currentPose, double dtSeconds) {
    return new Pose2d(currentPose.getX() + (speeds.vxMetersPerSecond * dtSeconds),
        currentPose.getY() + (speeds.vyMetersPerSecond * dtSeconds),
        currentPose.getRotation().plus(new Rotation2d(speeds.omegaRadiansPerSecond * dtSeconds)));
    
  }

}