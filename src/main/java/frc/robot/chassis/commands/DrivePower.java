package frc.robot.chassis.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.controller.CommandController;

public class DrivePower extends Command {
    private final Chassis chassis;
    private final CommandController controller;

    public DrivePower(Chassis chassis, CommandController commandController) {
        this.chassis = chassis;
        this.controller = commandController;
        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        chassis.setSteerPositions(-Math.PI*0.5);
    }

    @Override
    public void execute() {
        for (int i = 0; i < 4; i++) {
            chassis.setDrivePower(controller.getLeftY()*0.75, i);
        }
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }

    @Override
    public boolean isFinished() {
        return controller.rightButton().getAsBoolean();
    }
}
