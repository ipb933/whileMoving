// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.chassis.Mk5nConstants;
import frc.demacia.utils.controller.CommandController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DrivePower extends Command {
  Chassis chassis;
  CommandController controller;
  public DrivePower(Chassis chassis, CommandController controller) {
    this.chassis = chassis;
    this.controller = controller;
    addRequirements(chassis);
  }

  @Override
  public void initialize() {
    chassis.setSteerPositions(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.setDrivePower(controller.getLeftY());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
