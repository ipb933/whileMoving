// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.climb.constants.ClimbConstants;
import frc.robot.climb.subsystems.Climb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CalibrateClimb extends Command {
  private Climb climb;
  private Timer timer;
  private boolean hasStartedTimer;

  public CalibrateClimb(Climb climb) {
    this.climb = climb;
    this.timer = new Timer();
    this.hasStartedTimer = false;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.setLeverDuty(ClimbConstants.Calibration.POWER);
    if (climb.getCurrentAmpersLever() > ClimbConstants.Calibration.CURRENT_FOR_CALIBRATE) {
      if (!hasStartedTimer) {
        timer.start();
        hasStartedTimer = true;
      }
    } else {
      timer.stop();
      timer.reset();
      hasStartedTimer = false;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.setLeverDuty(0);
    climb.setLeverPosition(ClimbConstants.Calibration.POSITION_AFTER_CALIBRATION);
    climb.setCalibratedLever();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasStartedTimer && timer.hasElapsed(ClimbConstants.Calibration.TIME_TO_CONFIRM_CALIBRATION);
  }
}
