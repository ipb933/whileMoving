package frc.robot.target;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.demacia.utils.LookUpTable;
import frc.demacia.utils.chassis.Chassis;
import static frc.robot.target.TargetConstants.*;

public class Target3d {

    // give you the velocity needed to goal in every distance when not moving
    LookUpTable lookUpTable;

    // the chassis
    Chassis chassis;

    // shooter spees at the distanse if the robot is not moving
    DoubleSupplier targetShooterSpeedSupplier;
    double targetShooterSpeed;
    // shooter angle (up/down) at the distanse if the robot is not moving (radians)
    DoubleSupplier targetShooterAngleSupplier;
    double targetShooterAngle;
    // shooter Rotation (right/left) at the distanse if the robot is not moving (radians)
    DoubleSupplier targetShooterRotationSupplier;
    double targetShooterRotation;

    // the chassis speed in the shooter angle (right/left)
    double chassisRadialSpeed;
    // the chassis speed not in the shooter angle (right/left)
    double chassisTangentialSpeed;

    public Target3d(Chassis chassis, LookUpTable lookUpTable){
        this.chassis = chassis;
        this.lookUpTable = lookUpTable;
        targetShooterSpeedSupplier = () -> {
            return lookUpTable.get(distanceFromHubAfterTime(CYCLE_TIME))[0] * MOTOR_VEL_TO_BALL_VEL;
        };
        targetShooterAngleSupplier = () -> {
            return lookUpTable.get(distanceFromHubAfterTime(CYCLE_TIME))[1];
        };
        targetShooterRotationSupplier = () -> {
            return angleFromHubAfterTime(CYCLE_TIME);
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

    public ShootingValues getShootingValues() {
        targetShooterSpeed = targetShooterSpeedSupplier.getAsDouble();
        targetShooterAngle = targetShooterAngleSupplier.getAsDouble();
        targetShooterRotation = targetShooterRotationSupplier.getAsDouble();

        Pose2d futurePose = chassis.computeFuturePosition(CYCLE_TIME);

        Rotation3d endRotation = new Rotation3d(
            0, 
            targetShooterAngle, 
            targetShooterRotation + futurePose.getRotation().getRadians());
        Translation3d endValues = new Translation3d(
            targetShooterSpeed, 
            endRotation
        );
        
        ChassisSpeeds speeds = chassis.getChassisSpeedsFieldRel();
        Translation3d chassisSpeed = new Translation3d(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond, 
            0
        );
        Translation3d tangentialVelocity = new Translation3d(
            -speeds.omegaRadiansPerSecond*Math.sin(TargetConstants.SHOOTER_ANGLE_FROM_CENTER + futurePose.getRotation().getRadians())*TargetConstants.SHOOTER_DIST_FROM_CENTER, 
            speeds.omegaRadiansPerSecond*Math.cos(TargetConstants.SHOOTER_ANGLE_FROM_CENTER + futurePose.getRotation().getRadians())*TargetConstants.SHOOTER_DIST_FROM_CENTER, 
            0
        );
        chassisSpeed = chassisSpeed.plus(tangentialVelocity);

        Translation3d shooterValues = endValues.minus(chassisSpeed);

        double ShooterSpeed = shooterValues.getNorm();
        return new ShootingValues(
            ShooterSpeed / MOTOR_VEL_TO_BALL_VEL , 
            Math.asin(shooterValues.getZ() / ShooterSpeed) + (ShooterSpeed - targetShooterSpeed) * SPIN_CORRECTION_GAIN, 
            shooterValues.toTranslation2d().getAngle().getRadians() - chassis.getPose().getRotation().getRadians()
        );
    }
}