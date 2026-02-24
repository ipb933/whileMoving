// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.chassis.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveVelocity extends Command {
  Chassis chassis;
  double vel = 0;
  public DriveVelocity(Chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
    SmartDashboard.putData("DriveVelocityCommand", this);
  }
  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("wanted vel", ()->vel, (x)->vel=x);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.setSteerPositions(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.setDriveVelocities(vel);
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
