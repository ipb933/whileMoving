// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.demacia.utils.DemaciaUtils;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.chassis.DriveCommand;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import frc.demacia.utils.log.LogManager;
import frc.robot.Shooter.commands.HoodCalibrationCommand;
import frc.robot.Shooter.subsystem.Shooter;
import frc.robot.chassis.MK4iChassisConstants;
import frc.robot.chassis.commands.ResetModule;
import frc.robot.target.IdoCommand; 

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Sendable {

  public static boolean isComp = false;
  private static boolean hasRemovedFromLog = false;
  public static boolean isRed = false;
  Field2d field2d;
  Field2d questField2d;
  Chassis chassis;
  CommandController driverController = new CommandController(0, ControllerType.kPS5);
  Shooter shooter;

  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SmartDashboard.putData("RC", this);
    new DemaciaUtils(() -> getIsComp(), () -> getIsRed());
    chassis = new Chassis(MK4iChassisConstants.CHASSIS_CONFIG);
    shooter = new Shooter(chassis);

    SmartDashboard.putData("chassis/Reset Module Back Left", new ResetModule(chassis, 2, 0).ignoringDisable(true));
    SmartDashboard.putData("shooter/Hood calibration", new HoodCalibrationCommand(shooter));
    // Configure the trigger bindings
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  public static boolean isShooting = false;

  private void configureBindings() {
    DriveCommand driveCommand = new DriveCommand(chassis, driverController);
    chassis.setDefaultCommand(driveCommand);
    shooter.setDefaultCommand(new IdoCommand(chassis, shooter));
    driverController.rightButton().onTrue(new RunCommand(() -> {}, shooter));
    //   RobotContainer.isShooting = !RobotContainer.isShooting;
    //   if (isShooting) {
    //     CommandScheduler.getInstance().schedule(new InstantCommand(() -> new ShooterCommand(shooter, chassis).schedule()));
    //   }
    //   else {
    //     CommandScheduler.getInstance().schedule(new InstantCommand(() -> {}, shooter));
    //   }
    // }));
    driverController.upButton().onTrue(new InstantCommand(() -> driveCommand.setActiveToHub()));
    driverController.downButton().onTrue(new InstantCommand(() -> shooter.setIndexerPower(1)));
    driverController.leftBumper().onTrue(new InstantCommand(() -> shooter.setIndexerPower(0)));
  }

  public static boolean getIsRed() {
    return isRed;
  }

  public static void setIsRed(boolean isRed) {
    RobotContainer.isRed = isRed;
  }

  public static boolean getIsComp() {
    return isComp;
  }

  public static void setIsComp(boolean isComp) {
    RobotContainer.isComp = isComp;
    if (!hasRemovedFromLog && isComp) {
      hasRemovedFromLog = true;
      LogManager.removeInComp();
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("isRed", RobotContainer::getIsRed, RobotContainer::setIsRed);
    builder.addBooleanProperty("isComp", RobotContainer::getIsComp, RobotContainer::setIsComp);
    // builder.addDoubleProperty("Angle", () -> shooter.angle, (newAngle) ->
    // shooter.angle = newAngle);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}