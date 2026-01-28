package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.log.LogManager;
import frc.demacia.utils.log.LogEntryBuilder.LogLevel;
import frc.robot.shooter.ShooterConstans;
import frc.robot.shooter.subsystem.Shooter;
import frc.robot.target.ShootingValues;
import frc.robot.target.Target3d;
import static frc.robot.shooter.ShooterConstans.FlyWheelConstans.*;
import static frc.robot.shooter.ShooterConstans.HoodConstans.*;
import static frc.robot.shooter.ShooterConstans.IndexerConstans.*;
import static frc.robot.shooter.ShooterConstans.TurretConstans.TURRET_NAME;

public class ShooterCommand extends Command {
    private Shooter shooter;
    private Target3d target3d;


    public ShooterCommand(Chassis chassis, Shooter shooter) {
        this.shooter = shooter;
        target3d = new Target3d(chassis, ShooterConstans.SHOOTER_LOOKUP_TABLE);
        addRequirements(shooter);
    }

    @SuppressWarnings("unchecked")
    public void addLog(){
        
        LogManager.addEntry("is ready", () -> isReady())
        .withIsSeparated(true)
        .withLogLevel(LogLevel.LOG_AND_NT).build();
        LogManager.addEntry("is flywheel ready", () -> isFlywheelReady())
        .withIsSeparated(true)
        .withLogLevel(LogLevel.LOG_AND_NT).build();
        LogManager.addEntry("is hood ready", () -> isHoodReady())
        .withIsSeparated(true)
        .withLogLevel(LogLevel.LOG_AND_NT).build();
        LogManager.addEntry("is turret ready", () -> isTurretReady())
        .withIsSeparated(true)
        .withLogLevel(LogLevel.LOG_AND_NT).build();
    }

    @Override
    public void execute() {
        ShootingValues valuses = target3d.getShootingValues();

        shooter.setVelocity(FLYWHEEL_NAME, valuses.velocity());
        shooter.setHoodAngle(valuses.hoodAngle());
        shooter.setTurretAngle(valuses.turretAngle());

        if (isReady()){
            shooter.setVelocity(INDEXER_NAME, INDEXER_VEL);
        }
    }

    public boolean isFlywheelReady(){
        return shooter.getMotor(FLYWHEEL_NAME).getCurrentVelocity() < 0.2;
    }

    public boolean isHoodReady(){
        return shooter.getMotor(HOOD_NAME).getCurrentAngle() < 0.01;
    }

    public boolean isTurretReady(){
        return shooter.getMotor(TURRET_NAME).getCurrentAngle() < 0.01;
    }

    public boolean isReady(){
        return isFlywheelReady() &&
        isHoodReady() &&
        isTurretReady();
    }
}
