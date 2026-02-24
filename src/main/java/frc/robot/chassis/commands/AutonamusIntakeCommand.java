package frc.robot.chassis.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.vision.ObjectPose;

/**
 * Autonomous intake command that drives toward a game piece while automatically
 * rotating to the correct angle for intake by arrival time.
 */
public class AutonamusIntakeCommand extends Command {
  private Chassis chassis;
  private ObjectPose objectPose;
  
  // Tunable parameters
  private static final double MAX_DRIVE_SPEED = 2.5; // m/s - max approach speed
  private static final double MIN_DRIVE_SPEED = 0.3; // m/s - minimum speed to maintain control
  private static final double APPROACH_DISTANCE = 0.5; // meters - slow down when this close
  private static final double FINISH_THRESHOLD = 0.15; // meters - command finishes when this close
  private static final double MAX_ANGULAR_VELOCITY = 3.0; // rad/s - max rotation speed
  private static final double ANGULAR_KP = 2.0; // Proportional gain for angle correction
  
  private boolean hasValidTarget = false;
  
  public AutonamusIntakeCommand(Chassis chassis, ObjectPose objectPose) {
    this.chassis = chassis;
    this.objectPose = objectPose;
    addRequirements(chassis);

    
  }

  @Override
  public void initialize() {
    hasValidTarget = false;
  }

  @Override
  public void execute() {
    // Get current robot pose and object pose
    Pose2d robotPose = chassis.getPose();
    Pose2d targetPose = objectPose.getPose2d();
    
    // Check if we have a valid target
    if (targetPose.equals(Pose2d.kZero)) {
      chassis.setVelocities(new ChassisSpeeds(0, 0, 0));
      hasValidTarget = false;
      return;
    }
    
    hasValidTarget = true;
    
    // Calculate vector from robot to object
    Translation2d robotToObject = targetPose.getTranslation().minus(robotPose.getTranslation());
    double distance = robotToObject.getNorm();
    
    // Calculate required angle to face the object (for intake alignment)
    Rotation2d targetAngle = robotToObject.getAngle();
    
    // Calculate current angle error
    double angleError = targetAngle.minus(robotPose.getRotation()).getRadians();
    // Normalize angle error to [-π, π]
    angleError = Math.atan2(Math.sin(angleError), Math.cos(angleError));
    
    // Calculate linear velocity magnitude based on distance
    double linearSpeed;
    if (distance > APPROACH_DISTANCE) {
      // Full speed when far away
      linearSpeed = MAX_DRIVE_SPEED;
    } else {
      // Slow down proportionally as we approach
      linearSpeed = MathUtil.clamp(
        MIN_DRIVE_SPEED + (MAX_DRIVE_SPEED - MIN_DRIVE_SPEED) * (distance / APPROACH_DISTANCE),
        MIN_DRIVE_SPEED,
        MAX_DRIVE_SPEED
      );

      SmartDashboard.putNumber("distens", distance);
      SmartDashboard.putNumber("angle", targetAngle.getDegrees());
    }
    
    // Calculate time to reach target (assuming constant velocity)
    double timeToTarget = distance / linearSpeed;
    
    // Calculate required angular velocity to reach correct angle by arrival time
    double requiredOmega;
    if (timeToTarget > 0.05) { // Avoid division by very small numbers
      requiredOmega = angleError / timeToTarget;
    } else {
      // If very close, use proportional control for final alignment
      requiredOmega = angleError * ANGULAR_KP;
    }
    
    // Clamp angular velocity to maximum
    requiredOmega = MathUtil.clamp(requiredOmega, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    
    // Create velocity vector pointing toward target
    double vx = linearSpeed * targetAngle.getCos();
    double vy = linearSpeed * targetAngle.getSin();
    
    // Create field-relative chassis speeds
    ChassisSpeeds fieldSpeeds = new ChassisSpeeds(-vx, -vy, requiredOmega);
    
    // Send velocities to chassis
    chassis.setVelocities(fieldSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when command ends
    chassis.setVelocities(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    // Finish when we're close enough and have a valid target
    if (!hasValidTarget) {
      return false;
    }
    
    Pose2d robotPose = chassis.getPose();
    Pose2d targetPose = objectPose.getPose2d();
    
    if (targetPose.equals(Pose2d.kZero)) {
      return false;
    }
    
    double distance = targetPose.getTranslation().minus(robotPose.getTranslation()).getNorm();
    return distance < FINISH_THRESHOLD;
  }
}