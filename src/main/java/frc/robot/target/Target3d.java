package frc.robot.target;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.demacia.utils.LookUpTable;
import frc.demacia.utils.chassis.Chassis;
import frc.robot.Shooter.utils.ShooterUtils;

import static frc.robot.target.TargetConstants.*;

public class Target3d {

    // give you the velocity needed to goal in every distance when not moving
    LookUpTable lookUpTable;

    // the chassis
    Chassis chassis;

    // shooter spees at the distanse if the robot is not moving
    DoubleSupplier targetSpeedSupplier;
    double targetSpeed;
    // shooter angle (up/down) at the distanse if the robot is not moving (radians)
    DoubleSupplier targetHoodAngleSupplier;
    double targetHoodAngle;
    // shooter Rotation (right/left) at the distanse if the robot is not moving (radians)
    DoubleSupplier targetTurretAngleSupplier;
    double targetTurretAngle;

    // the chassis speed in the shooter angle (right/left)
    double chassisRadialSpeed;
    // the chassis speed not in the shooter angle (right/left)
    double chassisTangentialSpeed;

    public Target3d(Chassis chassis, LookUpTable lookUpTable){
        this.chassis = chassis;
        this.lookUpTable = lookUpTable;
        targetSpeedSupplier = () -> {
            return lookUpTable.get(distanceFromHubAfterTime(CYCLE_TIME))[0] * MOTOR_VEL_TO_BALL_VEL;
        };
        targetHoodAngleSupplier = () -> {
            return lookUpTable.get(distanceFromHubAfterTime(CYCLE_TIME))[1];
        };
        targetTurretAngleSupplier = () -> {
            return angleFromHubAfterTime(CYCLE_TIME);
        };
    }

    private Translation2d getTurretFuturePosition(double sec) {
        ChassisSpeeds robotSpeeds = chassis.getChassisSpeedsFieldRel();
        Pose2d futurePose = ShooterUtils.computeFuturePosition(robotSpeeds, chassis.getPose(), sec);
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
        double turretAngle = TargetConstants.hubPos.minus(turretPos).getAngle().getRadians();
    
        ChassisSpeeds robotSpeeds = chassis.getChassisSpeedsFieldRel();
        Pose2d futurePose = ShooterUtils.computeFuturePosition(robotSpeeds, chassis.getPose(), sec);
        double chassisRotation = futurePose.getRotation().getRadians();
        return turretAngle /*- chassisRotation*/;
    }

    public ShootingValues getShootingValues() {
        targetSpeed = targetSpeedSupplier.getAsDouble();
        targetHoodAngle = targetHoodAngleSupplier.getAsDouble();
        targetTurretAngle = targetTurretAngleSupplier.getAsDouble();

        
        ChassisSpeeds robotSpeeds = chassis.getChassisSpeedsFieldRel();
        Pose2d futurePose = ShooterUtils.computeFuturePosition(robotSpeeds, chassis.getPose(), CYCLE_TIME);

        Rotation3d endRotation = new Rotation3d(
            0, 
            targetHoodAngle, 
            targetTurretAngle + futurePose.getRotation().getRadians());
        Translation3d endValues = new Translation3d(
            targetSpeed, 
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
        // + (ShooterSpeed - targetSpeed) * SPIN_CORRECTION_GAIN

        // option 1: do nothing, change chassis speed
        // double ShooterSpeed = shooterValues.getNorm();
        // return new ShootingValues(
        //     ShooterSpeed / MOTOR_VEL_TO_BALL_VEL, 
        //     Math.asin(shooterValues.getZ() / ShooterSpeed), 
        //     shooterValues.toTranslation2d().getAngle().getRadians() - chassis.getPose().getRotation().getRadians()
        // );
        
        // option 2: add corraction to speed, not using time
        // double ShooterSpeed = shooterValues.getNorm();
        // return new ShootingValues(
        //     ShooterSpeed / MOTOR_VEL_TO_BALL_VEL  + (ShooterSpeed - targetSpeed) * SPIN_CORRECTION_GAIN, 
        //     Math.asin(shooterValues.getZ() / ShooterSpeed), 
        //     shooterValues.toTranslation2d().getAngle().getRadians() - chassis.getPose().getRotation().getRadians()
        // );
        
        // option 3: add corraction to hood angle, not using time
        // double ShooterSpeed = shooterValues.getNorm();
        // return new ShootingValues(
        //     ShooterSpeed / MOTOR_VEL_TO_BALL_VEL, 
        //     Math.asin(shooterValues.getZ() / ShooterSpeed)  + (ShooterSpeed - targetSpeed) * SPIN_CORRECTION_GAIN, 
        //     shooterValues.toTranslation2d().getAngle().getRadians() - chassis.getPose().getRotation().getRadians()
        // );
        
        // option 4: remove y speed
        // double ShooterSpeed = shooterValues.getNorm();
        // shooterValues = shooterValues.minus(new Translation3d(
        //     0, 
        //     0, 
        //     SPIN_CORRECTION_GAIN * (ShooterSpeed - targetSpeed) * targetSpeed *
        //     lookUpTable.get(distanceFromHubAfterTime(CYCLE_TIME))[2] // get time to be above hub 
        // ));
        // ShooterSpeed = shooterValues.getNorm();
        // return new ShootingValues(
        //     ShooterSpeed / MOTOR_VEL_TO_BALL_VEL, 
        //     Math.asin(shooterValues.getZ() / ShooterSpeed), 
        //     shooterValues.toTranslation2d().getAngle().getRadians() - chassis.getPose().getRotation().getRadians()
        // );
        
        // option 5: remove y speed and no t
        double ShooterSpeed = shooterValues.getNorm();
        shooterValues = shooterValues.minus(new Translation3d(
            0, 
            0, 
            SPIN_CORRECTION_GAIN * (ShooterSpeed - targetSpeed) * targetSpeed *
            targetSpeed * Math.cos(Math.asin(shooterValues.getZ() / ShooterSpeed)) *  distanceFromHubAfterTime(CYCLE_TIME)// time to be above hub 
        ));
        ShooterSpeed = shooterValues.getNorm();
        return new ShootingValues(
            ShooterSpeed / MOTOR_VEL_TO_BALL_VEL, 
            Math.asin(shooterValues.getZ() / ShooterSpeed), 
            shooterValues.toTranslation2d().getAngle().getRadians()/* - futurePose.getRotation().getRadians()*/
        );
    }
}