// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.motors.TalonSRXMotor;
import frc.demacia.utils.sensors.DigitalEncoder;
import frc.robot.climb.constants.ClimbConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climb extends SubsystemBase {
  private TalonSRXMotor armsMotor;
  private TalonFXMotor leverMotor;
  private DigitalEncoder digitalEncoder;
  private boolean hasClibaratedLever = false;

  /** Creates a new Climb. */
  public Climb() {
    armsMotor = new TalonSRXMotor(ClimbConstants.ARMS_MOTOR_CONFIG);
    leverMotor = new TalonFXMotor(ClimbConstants.LEVER_MOTOR_CONFIG);
    digitalEncoder = new DigitalEncoder(ClimbConstants.DIGITAL_ENCODER_CONFIG);
    // leverMotor.setPosition(0);
    SmartDashboard.putData("reset motor position",
        new InstantCommand(() -> leverMotor.setPosition(0)).ignoringDisable(true));
    SmartDashboard.putData("Climb", this);
  }

  public boolean hasCalibratedLever() {
    return hasClibaratedLever;
  }
  public void setCalibratedLever() {
    this.hasClibaratedLever = true;
  }
  public void setLeverPosition(double position) {
    leverMotor.setPosition(position);
  }

  public void checkElectronics() {
    armsMotor.checkElectronics();
    leverMotor.checkElectronics();
    digitalEncoder.checkElectronics();
  }

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Arms Current", this::getCurrentAmpersArms, null);
    builder.addDoubleProperty("Lever Angle", this::getAngleLever, null);
    builder.addDoubleProperty("Encoder Angle", this::getArmEncoderAngle, null);
    builder.addDoubleProperty("", null, null);
  }

  public void leverClimb() {
    if (getAngleLever() < ClimbConstants.ANGLE_LEVER_MID) {
      setLeverDuty(ClimbConstants.powerMid);
    } else if (getAngleLever() < ClimbConstants.ANGLE_LEVER_OPEN) {
      setLeverDuty(ClimbConstants.powerOpen);
    } else {
      leverMotor.stop();
    }
  }

  public void stateClose() {
    if (getArmEncoderAngle() != ClimbConstants.ARMS_ANGLE_CLOSED
        || getAngleLever() != ClimbConstants.ANGLE_LEVER_CLOSED) {
      setArmsAngle(ClimbConstants.ARMS_ANGLE_CLOSED);
      setLeverAngle(ClimbConstants.ANGLE_LEVER_CLOSED);
    }
  }

  public void setArmsDuty(double power) {
    armsMotor.setDuty(power);
  }

  public void setLeverDuty(double power) {
    leverMotor.setDuty(power);
  }

  public void stopArms() {
    armsMotor.stop();
  }

  public void stopLever() {
    leverMotor.stop();
  }

  public double getAngleLever() {
    return leverMotor.getCurrentAngle();
  }

  public void setArmsAngle(double angle) {
    angle = MathUtil.clamp(angle, 0, ClimbConstants.ANGLE_LEVER_CLOSED);
    double error = MathUtil.angleModulus(angle - getArmEncoderAngle());
    armsMotor.setVoltage(error * ClimbConstants.ARMS_KP);
  }

  public void setLeverAngle(double angle) {

    leverMotor.setPositionVoltage(angle);
  }

  public void resetLeverEncoder() {
    leverMotor.setEncoderPosition(0);
  }

  public double getCurrentAmpersArms() {
    return armsMotor.getCurrentCurrent();
  }

  public double getCurrentAmpersLever() {
    return leverMotor.getCurrentCurrent();
  }

  public double getArmEncoderAngle() {
    return MathUtil.angleModulus(MathUtil.angleModulus(digitalEncoder.get()) - ClimbConstants.ARMS_OFFSET);
  }

  public Pose2d getTargetClimbPose(boolean isRed, boolean isRightClimb) {
    if (isRed) {
      return isRightClimb ? ClimbConstants.targetRightSideRed : ClimbConstants.targetLeftSideRed;
    } else {
      return isRightClimb ? ClimbConstants.targetRightSideBlue : ClimbConstants.targetLeftSideBlue;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}