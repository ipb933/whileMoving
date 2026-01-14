package frc.robot.target;

import edu.wpi.first.math.geometry.Translation2d;

public final class TargetConstants {
  // hub pos
  public static final Translation2d hubPos = new Translation2d(0, 0); //TODO
  // time from sensor for the time the ball got his start velocity from the robot
  public static final double VELOCITY_TIME = 0; //TODO
  // time from sensor for the time the ball got his angle from the robot
  public static final double ANGLE_TIME = 0; //TODO
  // how much thimr is a cycle
  public static final double CYCLE_TIME = 0.02;
  // time from sensor for the time the ball got his rotation from the robot
  public static final double ROTATION_TIME = 0; //TODO
  public static final double MOTOR_VEL_TO_BALL_VEL = 1; //TODO

  public static final double SHOOTER_DIST_FROM_CENTER = 0; //TODO
  public static final double SHOOTER_ANGLE_FROM_CENTER = 0; //TODO

  /**
   * A correction factor for the aerodynamic lift (Magnus Effect).
   * When shooting on the move, the required exit velocity changes, which changes the flywheel RPM.
   * Different RPMs create different backspin amounts, altering the ball's lift compared to the static LookupTable.
   * This gain adjusts the angle setpoint linearly based on the deviation from the static velocity.
   */
  public static final double SPIN_CORRECTION_GAIN = 0; //TODO
}
