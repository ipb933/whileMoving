// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter.commands;

import frc.demacia.utils.mechanisms.CalibratinCommand;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.robot.shooter.ShooterConstans.HoodConstans;
import frc.robot.shooter.ShooterConstans.IndexerConstans;
import frc.robot.shooter.subsystem.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodCalibrationCommand extends CalibratinCommand {
  /** Creates a new shooterCommand. */

  public HoodCalibrationCommand(Shooter shooter) {
    super(shooter, 
    IndexerConstans.INDEXER_NAME, 
    0.1, 
    () -> ((LimitSwitch) shooter.getSensor(HoodConstans.LIMIT_SWITCH_NAME)).get(), 
    HoodConstans.MAX_ANGLE_HOOD);
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }
}
