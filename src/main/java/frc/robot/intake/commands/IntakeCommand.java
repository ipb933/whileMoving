// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotCommon;
import frc.robot.intake.IntakeConstants;
import frc.robot.intake.subsystems.IntakeSubsystem;

/**
 * This command activates the intake by the value of the
 * {@link RobotCommon.RobotStates}
 */
public class IntakeCommand extends Command {

  /** The Intake Subsystem */
  private IntakeSubsystem intakeSubsystem;

  private double power;

  /**
   * Creates a new Intake Command
   * 
   * @param intakeSubsystem the intake subsystem of the Robot Container
   */
  public IntakeCommand(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);

    power = 0;

    SmartDashboard.putData("Intake Command", this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Intake Power", this::getPower, this::setPower);
  }

  /**
   * activates Intake based on the current state
   */
  @Override
  public void execute() {

    switch (RobotCommon.currentState) { // ShootWithIntake, ShootWithoutIntake, DriveWhileIntake, Drive, PrepareClimb,
                                        // Climb, GetOffClimb
      case HubWithAutoIntake, HubWithoutAutoIntake, DeliveryWithAutoIntake, DeliveryWithoutAutoIntake:
        // intake
        intakeSubsystem.setDutyIntake(IntakeConstants.MAX_POWER);

        // indexers

        // battery
        // if (shinuaSubsystem.isAtMax(Math.toRadians(15))) {
        // batteryPower = -IntakeConstants.MAX_POWER;
        // } else if(shinuaSubsystem.isAtMin(Math.toRadians(15))){
        // batteryPower = IntakeConstants.MAX_POWER;
        // }
        // shinuaSubsystem.setPowerBattery(batteryPower);
        break;

      case DriveAutoIntake:
        // intake
        intakeSubsystem.setDutyIntake(0.8);

        break;
      case Test:
        intakeSubsystem.setDutyIntake(power);
        break;

      default:
        intakeSubsystem.stopIntake();
    }
  }

  /**
   * At the end of the command disable the intake
   */
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();
  }

  /**
   * never stop
   */
  @Override
  public boolean isFinished() {
    return false;
  }

  public IntakeSubsystem getIntakeSubsystem() {
    return intakeSubsystem;
  }

  public void setIntakeSubsystem(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
  }

  public double getPower() {
    return power;
  }

  public void setPower(double power) {
    this.power = power;
  }
}
