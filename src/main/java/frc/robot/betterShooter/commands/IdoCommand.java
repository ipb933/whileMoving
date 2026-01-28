package frc.robot.betterShooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.robot.betterShooter.ShooterConstans;
import frc.robot.betterShooter.subsystem.Shooter;
import frc.robot.target.ShootingValues;
import frc.robot.target.Target3d;

public class IdoCommand extends Command {
    private Shooter shooter;
    private Target3d target3d;


    public IdoCommand(Chassis chassis, Shooter shooter) {
        this.shooter = shooter;
        target3d = new Target3d(chassis, ShooterConstans.SHOOTER_LOOKUP_TABLE);
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        ShootingValues valuses = target3d.getShootingValues();

        shooter.setVelocity(0, valuses.velocity());
        shooter.setAngle(1, valuses.hoodAngle());
        shooter.setAngle(2, valuses.turretAngle());
    }
}
