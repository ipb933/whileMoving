// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.robot.intake.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  //the motor value
  private TalonFXMotor motorIntake;
  

  public IntakeSubsystem() {
    motorIntake = new TalonFXMotor(IntakeConstants.INTAKE_CONFIG);

    SmartDashboard.putData("Intkae", this);

    SmartDashboard.putData("Intake/Motor/set coast", new InstantCommand(() -> setNeutralMode(false)).ignoringDisable(true));
    SmartDashboard.putData("Intake/Motor/set brake", new InstantCommand(() -> setNeutralMode(true)).ignoringDisable(true));
  }

  public void checkElectronics() {
    motorIntake.checkElectronics();
  }

  public void setNeutralMode(boolean isBrake) {
    motorIntake.setNeutralMode(isBrake);
  }

  public void setDutyIntake(double pow) {
    motorIntake.setDuty(pow);
  }

  public void stopIntake() {
    motorIntake.stop();
  }
}
