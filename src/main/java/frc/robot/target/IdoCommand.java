package frc.robot.target;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.robot.Shooter.ShooterConstans;
import frc.robot.Shooter.subsystem.Shooter;

public class IdoCommand extends Command {
    private Chassis chassis;
    private Shooter shooter;
    private Target3d target3d;


    public IdoCommand(Chassis chassis, Shooter shooter) {
        this.chassis = chassis;
        this.shooter = shooter;
        target3d = new Target3d(chassis, ShooterConstans.SHOOTER_LOOKUP_TABLE);
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        ShootingValues valuses = target3d.getShootingValues();

        chassis.setTargetAngle(valuses.turretAngle());
        shooter.setFlywheelVel(valuses.velocity());
        shooter.setHoodAngle(valuses.hoodAngle());


    }
}
