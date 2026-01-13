package frc.robot.target;

import java.util.ArrayDeque;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.LookUpTable;
import frc.demacia.utils.chassis.Chassis;

public class Target2d extends SubsystemBase{

    // timer list to know befor how much time every fuel (ball) was in the sensor' used to set velocity
    ArrayDeque<Timer> velocityTimes;
    // timer list to know befor how much time every fuel was in the sensor' used to set rotation
    ArrayDeque<Timer> rotationTimes;

    // give you the velocity needed to goal in every distance when not moving
    LookUpTable lookUpTable;
    // turn on when sensor see fuel
    BooleanSupplier isFuel;
    boolean lastIsFuel;

    // the chassis
    Chassis chassis;

    // shooter spees at the distanse if the robot is not moving
    DoubleSupplier baseShooterSpeedSupplier;
    double baseShooterSpeed;
    // shooter angle (right/left) at the distanse if the robot is not moving (radians)
    DoubleSupplier baseShooterRotationSupplier;
    double baseShooterRotation;

    // the chassis speed in the shooter angle (right/left)
    double chassisRadialSpeed;
    // the chassis speed not in the shooter angle (right/left)
    double chassisTangentialSpeed;

    public Target2d(Chassis chassis, LookUpTable lookUpTable, BooleanSupplier isFuel){
        this.chassis = chassis;
        this.lookUpTable = lookUpTable;
        this.isFuel = isFuel;
        velocityTimes = new ArrayDeque<>();
        rotationTimes = new ArrayDeque<>();
        baseShooterSpeedSupplier = () -> {
            if (velocityTimes.isEmpty()){
                return lookUpTable.get(distanceFromHubAfterTime(TargetConstants.VELOCITY_TIME))[0] * TargetConstants.MOTOR_VEL_TO_BALL_VEL;
            }
            else {
                return lookUpTable.get(distanceFromHubAfterTime(TargetConstants.VELOCITY_TIME - velocityTimes.peek().get()))[0] * TargetConstants.MOTOR_VEL_TO_BALL_VEL;
            }
        };
        baseShooterRotationSupplier = () -> {
            if (rotationTimes.isEmpty()){
                return angleFromHubAfterTime(TargetConstants.ROTATION_TIME);
            }
            else {
                return angleFromHubAfterTime(TargetConstants.ROTATION_TIME - rotationTimes.peek().get());
            }
        };
    }

    private Translation2d getTurretFuturePosition(double sec) {
        Pose2d futurePose = chassis.computeFuturePosition(sec);
        double mountingAngle = TargetConstants.SHOOTER_ANGLE_FROM_CENTER + futurePose.getRotation().getRadians();
        
        return futurePose.getTranslation().plus(new Translation2d(
            TargetConstants.SHOOTER_DIST_FROM_CENTER * Math.cos(mountingAngle),
            TargetConstants.SHOOTER_DIST_FROM_CENTER * Math.sin(mountingAngle)
        ));
    }

    public double distanceFromHubAfterTime(double sec) {
        return TargetConstants.hubPos.getDistance(getTurretFuturePosition(sec));
    }

    public double angleFromHubAfterTime(double sec) {
        Translation2d turretPos = getTurretFuturePosition(sec);
        double fieldRelativeAngle = TargetConstants.hubPos.minus(turretPos).getAngle().getRadians();
    
        double chassisRotation = chassis.computeFuturePosition(sec).getRotation().getRadians();
        return fieldRelativeAngle - chassisRotation;
    }

    public double[] getValues() {
        baseShooterSpeed = baseShooterSpeedSupplier.getAsDouble();
        baseShooterRotation = baseShooterRotationSupplier.getAsDouble();

        Pose2d futurePose = chassis.computeFuturePosition(
            rotationTimes.isEmpty() ? TargetConstants.ROTATION_TIME : TargetConstants.ROTATION_TIME - rotationTimes.peek().get()
        );

        Rotation2d endRotation = new Rotation2d(baseShooterRotation + futurePose.getRotation().getRadians());
        Translation2d endValues = new Translation2d(
            baseShooterSpeed, 
            endRotation
        );
        
        ChassisSpeeds speeds = chassis.getChassisSpeedsFieldRel();
        Translation2d chassisSpeed = new Translation2d(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond
        );
        Translation2d tangentialVelocity = new Translation2d(
            -speeds.omegaRadiansPerSecond*Math.sin(TargetConstants.SHOOTER_ANGLE_FROM_CENTER + futurePose.getRotation().getRadians())*TargetConstants.SHOOTER_DIST_FROM_CENTER, 
            speeds.omegaRadiansPerSecond*Math.cos(TargetConstants.SHOOTER_ANGLE_FROM_CENTER + futurePose.getRotation().getRadians())*TargetConstants.SHOOTER_DIST_FROM_CENTER
        );
        chassisSpeed = chassisSpeed.plus(tangentialVelocity);

        Translation2d shooterValues = endValues.minus(chassisSpeed);

        return new double[] {
            shooterValues.getNorm() / TargetConstants.MOTOR_VEL_TO_BALL_VEL, //velocity
            shooterValues.getAngle().getRadians() - chassis.getPose().getRotation().getRadians() //rotation
        };
    }

    public void periodic(){
        if (isFuel.getAsBoolean() && !lastIsFuel){
            lastIsFuel = true;
            velocityTimes.add(new Timer());
            velocityTimes.peek().start();
            rotationTimes.add(new Timer());
            rotationTimes.peek().start();
        }
        else if (!isFuel.getAsBoolean() && lastIsFuel) {
            lastIsFuel = false;
        }
        if (!velocityTimes.isEmpty()){
            if (velocityTimes.peek().hasElapsed(TargetConstants.VELOCITY_TIME)){
                velocityTimes.poll();
            }
        }
        if (!rotationTimes.isEmpty()){
            if (rotationTimes.peek().hasElapsed(TargetConstants.ROTATION_TIME)){
                rotationTimes.poll();
            }
        }
    }
}