// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// bft-pgmc-wgo
package frc.robot;

import static edu.wpi.first.units.Units.Hertz;
import static frc.robot.Constants.*;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.FrequencyUnit;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import frc.demacia.utils.leds.LedManager;
import frc.demacia.utils.log.LogManager;
import frc.robot.RobotCommon.RobotStates;
import frc.demacia.utils.Data;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.chassis.DriveCommand;
import frc.demacia.utils.controller.CommandController;
import frc.demacia.utils.controller.CommandController.ControllerType;
import frc.robot.chassis.MK4iChassisConstants;
import frc.robot.chassis.RobotAChassisConstants;
import frc.robot.chassis.commands.DrivePower;
import frc.robot.chassis.commands.DriveVelocity;
import frc.robot.chassis.commands.SetModuleAngle;
import frc.robot.climb.commands.CalibrateClimb;
import frc.robot.climb.commands.ControllerClimb;
import frc.robot.climb.commands.StateBasedClimb;
import frc.robot.climb.subsystems.Climb;
import frc.robot.intake.commands.BatteryTest;
import frc.robot.intake.commands.IntakeCommand;
import frc.robot.intake.commands.ShinuaCommand;
import frc.robot.intake.subsystems.IntakeSubsystem;
import frc.robot.intake.subsystems.ShinuaSubsystem;
import frc.robot.leds.RobotALedStrip;
import frc.robot.shooter.commands.FlywheelTesting;
import frc.robot.shooter.commands.HoodTesting;
import frc.robot.shooter.commands.ShooterCommand;
import frc.robot.shooter.commands.ShooterTesting;
import frc.robot.shooter.subsystem.Shooter;
import frc.robot.turret.commands.TurretCalibration;
import frc.robot.turret.commands.TurretCommand;
import frc.robot.turret.commands.TurretFollow;
import frc.robot.turret.commands.TurretPower;
import frc.robot.turret.subsystems.Turret;

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

  public static Chassis chassis;
  CommandController driverController = new CommandController(0, ControllerType.kPS5);
  public static Turret turret;
  public static IntakeSubsystem intake;
  public static ShinuaSubsystem shinua;
  public static Shooter shooter;
  public static LedManager ledManager;
  public static RobotALedStrip leds;
  public static Climb climb;
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    intake = new IntakeSubsystem();
    shinua = new ShinuaSubsystem();
    shooter = new Shooter();
    ledManager = new LedManager();
    leds = new RobotALedStrip();
    // climb = new Climb();
    

    chassis = new Chassis(RobotAChassisConstants.CHASSIS_CONFIG);

    turret = Turret.getInstance();
    SmartDashboard.putData("RC", this);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    SmartDashboard.putData("Check Electronics", new InstantCommand(() -> {
      chassis.checkElectronics();
      intake.checkElectronics();
      shinua.checkElectronics();
      turret.checkElectronics();
      shooter.checkElectronics();
      // climb.checkElectronics();
    }).ignoringDisable(true));
    // addStatesToElasticForTesting();
    configureBindings();
    setUserButton();

    // Data.setFrequancyAll();
  }

  public void addStatesToElasticForTesting() {
    SendableChooser<RobotCommon.RobotStates> robotStateChooser = new SendableChooser<>();
    for (RobotCommon.RobotStates state : RobotCommon.RobotStates.class.getEnumConstants()) {
      robotStateChooser.addOption(state.name(), state);
    }
    robotStateChooser.onChange(state -> RobotCommon.currentState = state);
    SmartDashboard.putData("Robot State Chooser", robotStateChooser);

    SendableChooser<RobotCommon.Shifts> shiftsChooser = new SendableChooser<>();
    for (RobotCommon.Shifts state : RobotCommon.Shifts.class.getEnumConstants()) {
      shiftsChooser.addOption(state.name(), state);
    }
    shiftsChooser.onChange(state -> RobotCommon.currentShift = state);
    SmartDashboard.putData("Shifts Chooser", shiftsChooser);
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
  private void configureBindings() {
    chassis.setDefaultCommand(new DriveCommand(chassis, driverController));
    intake.setDefaultCommand(new IntakeCommand(intake));
    shinua.setDefaultCommand(new ShinuaCommand(shinua));
  
    // shooter.setDefaultCommand(new ShooterTesting(shooter));
    shooter.setDefaultCommand(new ShooterCommand(shooter, chassis));
    // climb.setDefaultCommand(new StateBasedClimb(climb, chassis));
    // driverController.rightButton().onTrue(new ControllerClimb(driverController, climb));
    // climb.setDefaultCommand(new ControllerClimb(driverController, climb));

    // turret.setDefaultCommand(new TurretFollow(turret, Field.HUB(true).getCenter().getTranslation(), chassis));
    SmartDashboard.putData("Activate Feeder", new StartEndCommand(() -> {
      shooter.setFeederPower(0.8);
    }, () -> {
      shooter.setFeederPower(0);
    }));

    
    // shooter.setDefaultCommand(new ShooterTesting(shooter));
    // turret.setDefaultCommand(new TurretPower(driverController)); 
    turret.setDefaultCommand(new TurretCommand(turret));
    // turret.setDefaultCommand(new TurretFollow(turret, Field.HUB(true).getCenter().getTranslation(), chassis));


    driverController.downButton().onTrue(RobotCommon.changeState(RobotStates.HubWithAutoIntake));
    driverController.upButton().onTrue(RobotCommon.changeState(RobotStates.DriveAutoIntake));
    driverController.rightButton().onTrue(RobotCommon.changeState(RobotStates.DeliveryWithAutoIntake));

    driverController.povDown().onTrue(RobotCommon.changeState(RobotStates.HubWithoutAutoIntake));
    driverController.povUp().onTrue(RobotCommon.changeState(RobotStates.Drive));
    driverController.povRight().onTrue(RobotCommon.changeState(RobotStates.DeliveryWithoutAutoIntake));

    driverController.rightBumper().onTrue(RobotCommon.changeState(RobotStates.IDLE));
    
    SmartDashboard.putData("Auto Drive", new RunCommand(()->chassis.setRobotRelVelocities(new ChassisSpeeds(2, 0, 0)), chassis));

    // SmartDashboard.putData("lever to zero", new InstantCommand(()->climb.setLeverAngle(0)));
    // SmartDashboard.putData("calibrate Climb", new CalibrateClimb(climb));
    SmartDashboard.putData("Reset Turret Position", new InstantCommand(()->turret.setEncoderPosition(0)).ignoringDisable(true));
    SmartDashboard.putData("Turret Calibration", new TurretCalibration(turret));
    SmartDashboard.putData("Config steer", new SetModuleAngle(chassis));
  }

  private void setUserButton() {
    new Trigger(() -> !DriverStation.isEnabled() && RobotController.getUserButton())
        .onTrue(new SetRobotNeutralMode(chassis, intake, shinua, turret, shooter).ignoringDisable(true));
  }



  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("is comp", () -> RobotCommon.isComp, (isComp) -> RobotCommon.isComp = isComp);
    builder.addBooleanProperty("is red", () -> RobotCommon.isRed, (isRed) -> RobotCommon.isRed = isRed);
    builder.addDoubleProperty("maxDriveVelocity", () -> chassis.getConfig().maxDriveVelocity, (max) -> chassis.getConfig().withMaxDriveVelocity(max));

    builder.addBooleanProperty("change is Robot Calibrated for testing", () -> RobotCommon.isRobotCalibrated,
        (isRobotCalibrated) -> RobotCommon.isRobotCalibrated = isRobotCalibrated);
    builder.addDoubleProperty("change Accuracy for testing", () -> RobotCommon.targetAccuracy,
        (targetAccuracy) -> RobotCommon.targetAccuracy = targetAccuracy);
  }

  static public void updateCommon() {
    Translation2d currentPoseFromHub = RobotCommon.currentRobotPose.getTranslation().minus(HUB_POS);
    RobotCommon.currentDistanceFromTarget = currentPoseFromHub.getNorm();
    RobotCommon.currentAngleFromTarget = currentPoseFromHub.getAngle().getRadians();
    RobotCommon.currentWantedTurretAngle = RobotCommon.currentWantedTurretAngle
        - RobotCommon.currentRobotPose.getRotation().getRadians();

    Translation2d futurePoseFromHub = RobotCommon.futureRobotPose.getTranslation().minus(HUB_POS);
    RobotCommon.futureDistanceFromTarget = futurePoseFromHub.getNorm();
    RobotCommon.futureAngleFromTarget = futurePoseFromHub.getAngle().getRadians();
    RobotCommon.futureWantedTurretAngle = RobotCommon.futureWantedTurretAngle
        - RobotCommon.futureRobotPose.getRotation().getRadians();

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