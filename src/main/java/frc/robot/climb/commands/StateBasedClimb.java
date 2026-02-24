// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climb.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.robot.RobotCommon;
import frc.robot.RobotCommon.RobotStates;
import frc.robot.climb.constants.ClimbConstants;
import frc.robot.climb.subsystems.Climb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StateBasedClimb extends Command {
    /** Creates a new StateBasedClimb. */
    Chassis chassis;
    Climb climb;
    Timer timer;
    private boolean IS_AT_BAR;
    private boolean IS_AT_GROUND;
    private boolean IS_RIGHT_CLIMB;
    private Pose2d chassisPose;
    private Translation2d difference;
    private ChassisSpeeds speed;
    private double headingDiff;
    private Pose2d targetPose;
    private double krakenPow;
    private double armsAngle;
    private boolean afterClimb;

    public StateBasedClimb(Climb climb, Chassis chassis) {
        this.climb = climb;
        this.chassis = chassis;
        this.timer = new Timer();
        this.krakenPow = 0;
        this.armsAngle = climb.getArmEncoderAngle();
        addRequirements(climb);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Kraken Power", () -> krakenPow, (value) -> krakenPow = value);
        builder.addDoubleProperty("Arms Angle", () -> armsAngle, (value) -> armsAngle = value);
        builder.addBooleanProperty("is right", () -> IS_RIGHT_CLIMB, (value) -> IS_RIGHT_CLIMB = value);
        builder.addBooleanProperty("is red", () -> RobotCommon.isRed, (value) -> RobotCommon.isRed = value);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        IS_AT_BAR = false;
        IS_AT_GROUND = false;
        afterClimb = false;

        IS_RIGHT_CLIMB = true; // need to set based on strategy
        timer.stop();
        timer.reset();

        krakenPow = 0;
        armsAngle = climb.getArmEncoderAngle();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (RobotCommon.currentState) {
            case PrepareClimb:
                climb.setArmsAngle(ClimbConstants.ANGLE_ARMS_RAISED);
                climb.setLeverAngle(ClimbConstants.ANGLE_LEVER_CLOSED);

                targetPose = climb.getTargetClimbPose(RobotCommon.isRed, IS_RIGHT_CLIMB);
                chassisPose = chassis.getPose();
                difference = targetPose.getTranslation().minus(chassisPose.getTranslation());
                headingDiff = targetPose.getRotation().minus(chassisPose.getRotation()).getRadians();
                speed = new ChassisSpeeds(difference.getX() * ClimbConstants.driveKp, difference.getY() * ClimbConstants.driveKp, headingDiff * ClimbConstants.rotationKp);
                // chassis.setVelocities(speed);
                if (difference.getNorm() < ClimbConstants.CHASSIS_TOLERANCE && Math.abs(headingDiff) <= ClimbConstants.CHASSIS_TOLERANCE) {
                    chassis.stop();
                }
                break;

            case Climb:
                climb.setArmsAngle(ClimbConstants.ANGLE_ARMS_LOWERED);

                if (climb.getArmEncoderAngle() >= ClimbConstants.ANGLE_ARMS_LOWERED) {
                    IS_AT_BAR = true;
                }

                if (IS_AT_BAR) {
                    climb.setArmsDuty(0.1);
                    climb.setLeverAngle(ClimbConstants.ANGLE_LEVER_OPEN);
                }
                break;
            case GetOffClimb:
                climb.setLeverAngle(ClimbConstants.ANGLE_LEVER_CLOSED);

                if (climb.getAngleLever() <= ClimbConstants.ANGLE_LEVER_CLOSED) {
                    IS_AT_GROUND = true;
                }

                if (IS_AT_GROUND) {
                    timer.start();
                    // chassis.setVelocities(new ChassisSpeeds(ClimbConstants.velocityToRaiseArmsAfterClimb, 0, 0));
                    climb.setArmsDuty(ClimbConstants.powerToRaiseArmsAfterClimb);
                    if (timer.hasElapsed(ClimbConstants.timeToRaiseArmsAfterClimb)) {
                        timer.stop();
                        timer.reset();
                        climb.stopArms();
                        afterClimb = true;
                    }
                    if(afterClimb){
                     timer.start();
                    // chassis.setVelocities(new ChassisSpeeds(ClimbConstants.velocityToGoBackAfterClimb,0,0));
                    }
                    if(timer.hasElapsed(ClimbConstants.timeToGoBackAfterClimb)){
                     timer.stop();
                     timer.reset();
                     chassis.stop();
                    }  

                 RobotCommon.currentState = RobotStates.Drive;

                }
                break;
            case Test:
                climb.setArmsAngle(armsAngle);
                climb.setLeverDuty(krakenPow);
                break;
            default:
                climb.stateClose();
                break;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}