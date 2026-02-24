package frc.robot.turret.TurretCommands;

import java.util.logging.LogManager;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.vision.TagPose;
import frc.robot.RobotCommon;
import frc.robot.RobotCommon.RobotStates;
import frc.robot.turret.Turret;


public class TurretCommand extends Command{
    private final Turret turret;
    private double wantedAngle = 0;
    
    public TurretCommand(Turret turret){
        this.turret = turret;
        addRequirements(turret);
        SmartDashboard.putData("Turret Command", this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Wanted Turret Angle", ()->wantedAngle, (x)->wantedAngle = x);
    }

    @Override
    public void initialize() {
        wantedAngle = Math.toDegrees(turret.getTurretAngle());
    }

    @Override
    public void execute() {
        // if(!turret.hasCalibrated() || !tag.isSeeTag(0,tag.GetDistFromCamera())) return;
        // turret.setPositionMotion(Math.toRadians(wantedAngle));
        switch(RobotCommon.currentState){
            // case ShootWithIntake,ShootWithoutIntake :
            //     turret.setPositionMotion(turret.getTurretToHubAngle());
            //     break;
            
            case HubWithAutoIntake, HubWithoutAutoIntake, DeliveryWithAutoIntake, DeliveryWithoutAutoIntake:
                turret.setPositionPID(RobotCommon.futureAngleFromTargetRobotRelative);
                // frc.demacia.utils.log.LogManager.log("Turret angle: " + RobotCommon.futureAngleFromTargetRobotRelative);
                break;

            case Drive, DriveAutoIntake:
                turret.setPositionPID(0);
                break;
            case Test:
                turret.setPositionPID(Math.toRadians(wantedAngle));
                break;

            default:
                turret.stop();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
