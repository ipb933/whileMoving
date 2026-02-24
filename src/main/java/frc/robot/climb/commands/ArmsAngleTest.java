// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.climb.subsystems.Climb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmsAngleTest extends Command {
  double wantedAngle = 0;
  Climb climb;
  public ArmsAngleTest(Climb climb) {
    this.climb = climb;

    SmartDashboard.putData("Climb test command",this);
    addRequirements(climb);
  }

  
  @Override
  public void initSendable(SendableBuilder builder) {
      // TODO Auto-generated method stub
      super.initSendable(builder);
      builder.addDoubleProperty("Wanted arms angle", ()->wantedAngle, (x)->wantedAngle=x);
  }
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.setArmsAngle(Math.toRadians(wantedAngle));
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