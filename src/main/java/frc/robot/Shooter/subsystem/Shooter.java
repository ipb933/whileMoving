// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter.subsystem;

import edu.wpi.first.math.MathUtil;
import frc.demacia.utils.mechanisms.BaseMechanism;
import frc.demacia.utils.motors.MotorInterface;
import frc.demacia.utils.motors.TalonFXMotor;
import frc.demacia.utils.sensors.LimitSwitch;
import frc.demacia.utils.sensors.SensorInterface;
import static frc.robot.shooter.ShooterConstans.FlyWheelConstans.*;
import static frc.robot.shooter.ShooterConstans.HoodConstans.*;
import static frc.robot.shooter.ShooterConstans.TurretConstans.*;
import static frc.robot.shooter.ShooterConstans.IndexerConstans.*;

public class Shooter extends BaseMechanism {
  /** Creates a new shooter. */


  public Shooter() {
    super("shooter", 
    new MotorInterface[]{
      new TalonFXMotor(FLYWHEEL_CONFIG), 
      new TalonFXMotor(HOOD_CONFIG),
      new TalonFXMotor(null), 
      new TalonFXMotor(INDEXER_CONFIG)
    }, 
    new SensorInterface[]{
      new LimitSwitch(LIMIT_SWITCH_CONFIG)
    });
  }

  public void setHoodAngle(double angle){
    angle = MathUtil.clamp(angle, MIN_ANGLE_HOOD, MAX_ANGLE_HOOD);
    setAngle(HOOD_NAME, angle);
  }

  public void setTurretAngle(double angle){
    angle = MathUtil.clamp(angle, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);
    setAngle(TURRET_NAME, angle);
  }

  @Override
  public void periodic() {
      
  }
}
