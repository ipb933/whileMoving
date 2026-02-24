package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.intake.IntakeConstants;
import frc.robot.intake.subsystems.ShinuaSubsystem;

public class BatteryTest extends Command {

    private final ShinuaSubsystem shinua;
    private boolean isDirectionUp;

    public BatteryTest(ShinuaSubsystem shinuaSubsystem) {
        this.shinua = shinuaSubsystem;
        this.isDirectionUp = true;
        addRequirements(shinuaSubsystem);
    }

    @Override
    public void initialize() {
        isDirectionUp = true;
    }

    @Override
    public void execute() {
        if (shinua.isAtMax()) {
            isDirectionUp = false;
        }
        if (shinua.isAtMin()) {
            isDirectionUp = true;
        }

        shinua.setPowerBattery(isDirectionUp ? IntakeConstants.MAX_POWER : -IntakeConstants.MAX_POWER);
    }

    @Override
    public void end(boolean interrupted) {
        shinua.stop();
    }
}
