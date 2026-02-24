// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.vision.ObjectPose;
import frc.robot.intake.subsystems.IntakeSubsystem;

public class Intake {
  private Chassis chassis;
  private IntakeSubsystem intake;
  private ObjectPose objectPose;
  private CommandController controller;
  private final Translation2d chassisToIntakeOffset;
  // Fuel position from vision

  public Intake(Chassis chassis, IntakeSubsystem intake, ObjectPose objectPose,
      CommandController controller) {
    this.chassis = chassis;
    this.controller = controller;
    this.intake = intake;
    this.objectPose = objectPose;
    chassisToIntakeOffset = new Translation2d(0.2, -0.05);
  }

  public ChassisSpeeds AutoIntakeSpeeds() {
    Translation2d driverVelocityVectorRobotRel = new Translation2d(controller.getLeftY(), controller.getLeftX())
        .rotateBy(chassis.getGyroAngle().unaryMinus());
    double wantedVxRobotRel = (Math.min(Math.abs(driverVelocityVectorRobotRel.getX() * chassis.getConfig().maxDriveVelocity),
        2.5));
    Translation2d intakeToTarget = objectPose.getRobotToObject().minus(chassisToIntakeOffset);
    double angleToFix = Math.min(Math.abs(intakeToTarget.getAngle().getRadians() * 2), Math.toRadians(90))
        * Math.signum(intakeToTarget.getAngle().getRadians());
    return new ChassisSpeeds(-wantedVxRobotRel, -wantedVxRobotRel * Math.tan(angleToFix), 0);
  }

  public boolean isSeeFuel() {
    return objectPose.getRobotToObject().minus(chassisToIntakeOffset).getNorm() > 0.03
        && objectPose.getDistcameraToObject() > 0.01;
  }

  public void setDutyIntake(double pow){
    intake.setDutyIntake(pow);
  }
}