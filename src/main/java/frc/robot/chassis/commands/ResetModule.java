package frc.robot.chassis.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;

public class ResetModule extends Command {

    private final Chassis chassis;
    private final int id;
    private final double angle;

    public ResetModule(Chassis chassis, int id, double angle) {
        this.chassis = chassis;
        this.id = id;
        this.angle = angle;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        chassis.modules[id].steerMotor.setEncoderPosition(angle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
