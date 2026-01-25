// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter.subsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.demacia.vision.subsystem.Tag;
import frc.robot.Shooter.ShooterConstans;
import frc.robot.Shooter.utils.ShooterUtils;

public class Shooter extends SubsystemBase {
  /** Creates a new shooter. */

  private TalonFXMotor shooterMotor;
  private TalonFXMotor indexerMotor;
  private TalonFXMotor hoodMotor;
  private boolean hasCalibrated;

  private LimitSwitch limitSwitch;

  Chassis chassis;
  public double angle;

  public Shooter(Chassis chassis) {
    this.chassis = chassis;
    hoodMotor = new TalonFXMotor(ShooterConstans.HOOD_CONFIG);
    shooterMotor = new TalonFXMotor(ShooterConstans.SHOOTER_MOTOR_CONFIG);
    indexerMotor = new TalonFXMotor(ShooterConstans.INDEXER_CONFIG);
    this.hasCalibrated = false;
    this.limitSwitch = new LimitSwitch(ShooterConstans.LIMIT_SWITCH_CONFIG);
    SmartDashboard.putData("Shooter", this);
    hoodMotor.configPidFf(0);

  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("get angle", () -> getAngleHood(), null);
    builder.addDoubleProperty("get Vel", () -> getShooterVelocity(), null);
    builder.addBooleanProperty("Is At Limit", () -> isAtLimit(), null);
    builder.addBooleanProperty("Is Calibrated", () -> hasCalibrated, null);
  }

  public boolean isAtLimit() {
    return !limitSwitch.get();
  }

  public void setHoodMotorPosition(double position) {
    hoodMotor.setEncoderPosition(position);
  }

  public void setFlywheelVel(double speed) {
    shooterMotor.setVelocity(speed);
  }

  public double getShooterVelocity() {
    return shooterMotor.getVelocity().getValueAsDouble();
  }

  public void setPower(double power) {
    shooterMotor.set(power);
  }

  public void setHoodPower(double power) {
    hoodMotor.set(power);
  }

  public void setHoodAngle(double angle) {
    //this.angle = angle;
    if (!hasCalibrated) {
      hoodMotor.set(0);
      return;
    }
    
    angle = MathUtil.clamp(angle, ShooterConstans.MIN_ANGLE_HOOD, ShooterConstans.MAX_ANGLE_HOOD);
    hoodMotor.setPositionVoltage(angle);
  }

  public double getAngleHood() {
    return hoodMotor.getCurrentAngle();
  }

  public void setVelocitiesAndAngle(double vel, double angle) {
    //this.angle = angle;
    setFlywheelVel(vel);
    setHoodAngle(angle);
  }

  public boolean hasCalibrated() {
    return this.hasCalibrated;
  }

  public void setCalibrated() {
    this.hasCalibrated = true;
  }

  public double getLookUpTableVel(double distance) {
    return ShooterConstans.SHOOTER_LOOKUP_TABLE.get(distance)[0];
  }

  public double getLookUpTableAngle(double distance) {
    return ShooterConstans.SHOOTER_LOOKUP_TABLE.get(distance)[1];
  }

  public Translation3d getVelInVector(double vel) {
    return new Translation3d(vel, new Rotation3d(0, getAngleHood(), chassis.getGyroAngle().getRadians()));
  }

  public void setIndexerPower(double pow) {
    indexerMotor.set(pow);
  }

  public Pose2d RobotFucerPose(){
    return ShooterUtils.computeFuturePosition(chassis.getChassisSpeedsFieldRel(), chassis.getPose(), 0.02);
  }


  public boolean isShooterReady() {
    return Math.abs(shooterMotor.getClosedLoopError().getValueAsDouble()) < 0.2;
  }  

  public void stop() {
    shooterMotor.stopMotor();
  }

  // hub pose (i finde it with april tag)
  public Translation3d hubPose() {
    return new Translation3d(449.5 / 100, 370.84000000000003 / 100, 142.24 / 2);
  }

  // shooter pose on the robot
  public Translation3d ShooterPoseOnRobot() {
    return new Translation3d();
  }

  public Translation3d getShooterPosOnField() {
    return new Translation3d();
  }

  // get the distins from the shooter to the target
  public Translation3d getVectorToHubShoter() {
    return ShooterConstans.HUB_POSE_Translation3d.minus(getShooterPosOnField());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
