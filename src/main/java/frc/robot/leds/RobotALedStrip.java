package frc.robot.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.demacia.utils.leds.LedStrip;
import frc.robot.RobotCommon;
import frc.robot.RobotContainer;

public class RobotALedStrip extends LedStrip {
    
    public RobotALedStrip() {
        super("Robot A Strip", 14, RobotContainer.ledManager);
    }

    private Color getColorBasedOnState(RobotCommon.RobotStates state) {
        switch (RobotCommon.currentState) {
            case Climb:
                return Color.kOrangeRed;
            case DeliveryWithAutoIntake:
                return Color.kRed;
            case DeliveryWithoutAutoIntake:
                return Color.kDarkRed;
            case Drive:
                return Color.kWhiteSmoke;
            case DriveAutoIntake:
                return Color.kWhite;
            case GetOffClimb:
                return Color.kDarkOrange;
            case HubWithAutoIntake:
                return Color.kBlue;
            case HubWithoutAutoIntake:
                return Color.kAliceBlue;
            case IDLE:
                return Color.kPurple;
            case PrepareClimb:
                return Color.kBeige;
            case Test:
                return Color.kDeepPink;
            case Trench:
                return Color.kGreen;
            default:
                return Color.kBlack;
        }
    }

    public void changeColor(RobotCommon.RobotStates state) {
        CommandScheduler.getInstance().schedule(new RunCommand(() -> setBlink(getColorBasedOnState(state)), this).ignoringDisable(true).withTimeout(2).andThen(new RunCommand(() -> setColor(getColorBasedOnState(state)), this)));
    }
}
