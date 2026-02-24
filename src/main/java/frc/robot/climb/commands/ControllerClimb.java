// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.demacia.utils.controller.CommandController;
import frc.robot.climb.subsystems.Climb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ControllerClimb extends Command {
  CommandController contoller;
  Climb climb;
  private double joyright;
  private double joyleft;

  /** Creates a new ControllerClimb. */
  public ControllerClimb(CommandController controller, Climb climb) {
    this.contoller = controller;
    this.climb = climb;
    addRequirements(climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    joyright = contoller.getRightY();
    climb.setLeverDuty(joyright);

    joyleft = contoller.getLeftY() * 0.2;
    climb.setArmsDuty(joyleft);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.stopArms();
    climb.stopLever();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}