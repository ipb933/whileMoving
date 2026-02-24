package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.robot.intake.subsystems.IntakeSubsystem;
import frc.robot.intake.subsystems.ShinuaSubsystem;
import frc.robot.shooter.subsystem.Shooter;
import frc.robot.turret.Turret;

public class SetRobotNeutralMode extends Command {
    private final Chassis chassis;
    private final IntakeSubsystem intake;
    private final ShinuaSubsystem shinua;
    private final Turret turret;
    private final Shooter shooter;

    private boolean isBrake;

    public SetRobotNeutralMode(Chassis chassis, IntakeSubsystem intake, ShinuaSubsystem shinua, Turret turret, Shooter shooter) {
        this.chassis = chassis;
        this.intake = intake;
        this.shinua = shinua;
        this.turret = turret;
        this.shooter = shooter;

        isBrake = true;
    }

    @Override
    public void initialize() {
        isBrake = !isBrake;

        chassis.setNeutralMode(isBrake);
        intake.setNeutralMode(isBrake);
        shinua.setNeutralMode(isBrake);
        turret.setNeutralMode(isBrake);
        shooter.setNeutralMode(isBrake);
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
