// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.betterShooter.subsystem;

import frc.demacia.utils.mechanisms.BaseMechanism;
import frc.demacia.utils.motors.MotorInterface;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.demacia.utils.sensors.SensorInterface;
import frc.robot.betterShooter.ShooterConstans.FlyWheelConstans;
import frc.robot.betterShooter.ShooterConstans.HoodConstans;
import frc.robot.betterShooter.ShooterConstans.IndexerConstans;;

public class Shooter extends BaseMechanism {
  /** Creates a new shooter. */


  public Shooter() {
    super("shooter", 
    new MotorInterface[]{
      new TalonFXMotor(FlyWheelConstans.FLYWHEEL_CONFIG), 
      new TalonFXMotor(HoodConstans.HOOD_CONFIG), 
      new TalonFXMotor(IndexerConstans.INDEXER_CONFIG)
    }, 
    new SensorInterface[]{
      new LimitSwitch(HoodConstans.LIMIT_SWITCH_CONFIG)
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
