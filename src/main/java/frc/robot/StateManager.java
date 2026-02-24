package frc.robot;

import java.util.ArrayList;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class StateManager extends SubsystemBase {

    private static StateManager instance;
    private boolean ischangeAuto = false;

    public static StateManager getInstance() {
        if (instance == null) {
            instance = new StateManager();
        }
        return instance;
    }

    private RobotCommon.RobotStates lastState;

    private final EventLoop eventLoop;

    private StateManager() {
        lastState = RobotCommon.RobotStates.IDLE;
        eventLoop = new EventLoop();
    }

    public void setLastState(RobotCommon.RobotStates lastState) {
        this.lastState = lastState;
    }

    public RobotCommon.RobotStates getLastState() {
        return lastState;
    }

    public void addTrigger(BooleanSupplier onTrue, RobotCommon.RobotStates wantedState,
            RobotCommon.RobotStates... fromStates) {
        new Trigger(eventLoop, onTrue).and(() -> {
            for (RobotCommon.RobotStates state : fromStates) {
                if (state.equals(lastState))
                    return true;
            }
            return false;
        }).onTrue(new InstantCommand(() -> {
            RobotCommon.currentState = wantedState;
        }).ignoringDisable(true));
    }

    @Override
    public void periodic() {
        if (ischangeAuto) eventLoop.poll();        
    }
}
