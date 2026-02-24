package frc.robot.turret.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.odometry.RobotPose;
import frc.demacia.utils.chassis.Chassis;
import frc.robot.turret.subsystems.Turret;

public class TurretFollow extends Command {
    private final Turret turret;
    private final Translation2d target;
    private final Chassis chassis;

    public TurretFollow(Turret turret, Translation2d target, Chassis chassis) {
        this.turret = turret;
        this.target = target;
        this.chassis = chassis;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        
        turret.setPositionPID(MathUtil.angleModulus(target.minus(chassis.getPoseWithVelocity(0.04).getTranslation()).getAngle().minus(chassis.getPose().getRotation()).getRadians()));
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
    
}
