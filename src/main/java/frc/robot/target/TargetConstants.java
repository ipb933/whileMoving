package frc.robot.target;

import edu.wpi.first.math.geometry.Translation2d;

public final class TargetConstants {
  // hub pos
  public static final Translation2d hubPos = new Translation2d(11.265 + 0.5969, 4.023);
  // how much thimr is a cycle
  public static final double PREDICTING_TIME = 0.04;
  public static final double MOTOR_VEL_TO_BALL_VEL = 0.48;

  public static final double SHOOTER_DIST_FROM_CENTER = 0.15;
  public static final double SHOOTER_ANGLE_FROM_CENTER = 0;

  /**
   * A correction factor for the aerodynamic lift (Magnus Effect).
   * When shooting on the move, the required exit velocity changes, which changes the flywheel RPM.
   * Different RPMs create different backspin amounts, altering the ball's lift compared to the static LookupTable.
   * This gain adjusts the angle setpoint linearly based on the deviation from the static velocity.
   */
  public static final double SPIN_CORRECTION_GAIN = 0; //0.0001; //TODO
}
