package frc.robot.intake.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.motors.TalonSRXMotor;
import frc.robot.intake.IntakeConstants;

public class ShinuaSubsystem extends SubsystemBase {
  private TalonSRXMotor motorIndexerClose;
  private TalonSRXMotor motorIndexerFar;
  private TalonFXMotor motorBattery;
  private TalonFXMotor motorIndexerOnTop;

  public ShinuaSubsystem() {
    motorIndexerClose = new TalonSRXMotor(IntakeConstants.INDEXER_CLOSE_CONFIG);
    motorIndexerFar = new TalonSRXMotor(IntakeConstants.INDEXER_FAR_CONFIG);
    motorIndexerOnTop = new TalonFXMotor(IntakeConstants.INDEXER_ON_TOP_CONFIG);
    motorBattery = new TalonFXMotor(IntakeConstants.BATTERY_CONFIG);
    setEncoderPositionBattery(IntakeConstants.MIN_POSITION);
    motorBattery.configPidFf(0);

    SmartDashboard.putData("Shinua", this);
    SmartDashboard.putData("Shinua/Top/set coast", new InstantCommand(() -> motorIndexerOnTop.setNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putData("Shinua/Top/set brake", new InstantCommand(() -> motorIndexerOnTop.setNeutralMode(true)).ignoringDisable(true));
    SmartDashboard.putData("Shinua/Left/set coast", new InstantCommand(() -> motorIndexerFar.setNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putData("Shinua/Left/set brake", new InstantCommand(() -> motorIndexerFar.setNeutralMode(true)).ignoringDisable(true));
    SmartDashboard.putData("Shinua/Right/set coast", new InstantCommand(() -> motorIndexerClose.setNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putData("Shinua/Right/set brake", new InstantCommand(() -> motorIndexerClose.setNeutralMode(true)).ignoringDisable(true));
    SmartDashboard.putData("Shinua/Battery/set coast", new InstantCommand(() -> motorBattery.setNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putData("Shinua/Battery/set brake", new InstantCommand(() -> motorBattery.setNeutralMode(true)).ignoringDisable(true));
  }

  public void checkElectronics() {
    motorIndexerClose.checkElectronics();
    motorIndexerFar.checkElectronics();
    motorIndexerOnTop.checkElectronics();
    motorBattery.checkElectronics();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
    
      builder.addDoubleProperty("Batter Position", () -> motorBattery.getCurrentPosition(), null);
  }

  public void setNeutralMode(boolean isBrake) {
    motorBattery.setNeutralMode(isBrake);
    motorIndexerClose.setNeutralMode(isBrake);
    motorIndexerFar.setNeutralMode(isBrake);
    motorIndexerOnTop.setNeutralMode(isBrake);
  }

  public void setDutyIndexerClose(double pow) {
    motorIndexerClose.setDuty(pow);
  }

  public void stopIndexerClose() {
    motorIndexerClose.stop();
  }

  public void setDutyIndexerFar(double pow) {
    motorIndexerFar.setDuty(pow);
  }

  public void stopIndexerFar() {
    motorIndexerFar.stop();
  }

  public void setDutyIndexerOnTop(double pow) {
    motorIndexerOnTop.setDuty(pow);
  }

  public void stopIndexerOnTop() {
    motorIndexerOnTop.stop();
  }

  public void setEncoderPositionBattery(double position) {
    motorBattery.setEncoderPosition(position);
  }

  public void setPositionBattery(double position) {
    motorBattery.setPositionVoltage(position);
  }

  public void setPowerBattery(double power) {
    motorBattery.set(power);
  }

  public boolean isAtMin() {
    return IntakeConstants.MIN_POSITION >= motorBattery.getCurrentPosition();
  }

  public boolean isAtMax() {
    return IntakeConstants.MAX_POSITION <= motorBattery.getCurrentPosition();
  }

  public void stop() {
    motorBattery.stop();
    motorIndexerClose.stop();
    motorIndexerFar.stop();
    motorIndexerOnTop.stop();
  }
}