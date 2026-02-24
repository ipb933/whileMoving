package frc.robot.chassis.commands;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.demacia.utils.chassis.Chassis;
import frc.demacia.utils.chassis.SwerveModule;
import frc.demacia.utils.motors.CloseLoopParam;
import frc.demacia.utils.motors.TalonFXMotor;

public class SetModuleAngle extends Command {
    private final Chassis chassis;
    private final SwerveModule[] swerveModules;
    private TalonFXMotor[] motors;
    private CloseLoopParam pid;
    private Command configPidFf;
    private double position;

    public SetModuleAngle(Chassis chassis) {
        this.chassis = chassis;
        addRequirements(chassis);
        swerveModules = chassis.modules;
        motors = new TalonFXMotor[4];
        for (int i = 0; i < swerveModules.length; i++) {
            motors[i] = (TalonFXMotor) swerveModules[i].getSteerMotor();
        }
        pid = motors[0].getConfig().pid[0];
        configPidFf = new InstantCommand(() -> {
            Slot0Configs cfgs = new Slot0Configs();
            cfgs.kP = pid.kP();
            cfgs.kI = pid.kI();
            cfgs.kD = pid.kD();
            cfgs.kS = pid.kS();
            cfgs.kV = pid.kV();
            cfgs.kA = pid.kA();
            cfgs.kG = pid.kG();
            for (int i = 0; i < 4; i++) {
                motors[i].getConfigurator().apply(cfgs);
            }
        }).ignoringDisable(true);
        position = 0;
    }

    @Override
    public void initialize() {
        SmartDashboard.putData("Config Steer PID", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("PID+FF Config");

                builder.addDoubleProperty("KP", () -> pid.kP(), (double newValue) -> pid.setKP(newValue));
                builder.addDoubleProperty("KI", () -> pid.kI(), (double newValue) -> pid.setKI(newValue));
                builder.addDoubleProperty("KD", () -> pid.kD(), (double newValue) -> pid.setKD(newValue));
                builder.addDoubleProperty("KS", () -> pid.kS(), (double newValue) -> pid.setKS(newValue));
                builder.addDoubleProperty("KV", () -> pid.kV(), (double newValue) -> pid.setKV(newValue));
                builder.addDoubleProperty("KA", () -> pid.kA(), (double newValue) -> pid.setKA(newValue));
                builder.addDoubleProperty("KG", () -> pid.kG(), (double newValue) -> pid.setKG(newValue));

                builder.addBooleanProperty("Update", () -> configPidFf.isScheduled(),
                        value -> {
                            if (value) {
                                if (!configPidFf.isScheduled()) {
                                    configPidFf.schedule();
                                }
                            } else {
                                if (configPidFf.isScheduled()) {
                                    configPidFf.cancel();
                                }
                            }
                        });

                    builder.addDoubleProperty("Position", () -> position, (newPosition) -> position = newPosition);
            }
        });
    }

    @Override
    public void execute() {
        for (SwerveModule module : swerveModules) {
            module.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(position)));  
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.stop();
    }
}
