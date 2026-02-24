// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.chassis;

// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.demacia.kinematics.DemaciaKinematics;
import frc.demacia.odometry.DemaciaPoseEstimator;
import frc.demacia.odometry.RobotPose;
import frc.demacia.odometry.DemaciaPoseEstimator.OdometryObservation;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.demacia.utils.Utilities;
import frc.demacia.utils.sensors.Pigeon;
import frc.demacia.vision.ObjectPose;
import frc.demacia.vision.TagPose;
import frc.demacia.vision.subsystem.Quest;
import frc.demacia.vision.utils.Vision;
import frc.demacia.vision.utils.VisionConstants;
import frc.robot.RobotCommon;
import frc.robot.shooter.constants.ShooterConstans;
import frc.robot.shooter.utils.ShooterUtils;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.demacia.vision.utils.VisionConstants.*;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.ejml.simple.SimpleMatrix;

/**
 * Main swerve drive chassis controller.
 * 
 * <p>
 * Manages four swerve modules, odometry, and provides high-level drive control
 * with acceleration limiting and smooth motion profiling.
 * </p>
 * <p>
 * Manages four swerve modules, odometry, and provides high-level drive control
 * with acceleration limiting and smooth motion profiling.
 * </p>
 * 
 * <p>
 * <b>Features:</b>
 * </p>
 * <ul>
 * <li>Field-relative and robot-relative control</li>
 * <li>Pose estimation with vision integration</li>
 * <li>Smooth acceleration limiting</li>
 * <li>Path following capabilities</li>
 * <li>Auto-rotate to target angle</li>
 * </ul>
 * 
 * <p>
 * <b>Example Usage:</b>
 * </p>
 * 
 * <p>
 * <b>Example Usage:</b>
 * </p>
 * 
 * <pre>
 * ChassisConfig config = new ChassisConfig(
 *         "MainChassis",
 *         swerveModueles[] swerveModulesConfig,
 *         pigeonConfig,
 * );
 * 
 * Chassis chassis = new Chassis(config);
 * 
 * // Field-relative drive with acceleration limiting
 * chassis.setVelocitiesWithAccel(new ChassisSpeeds(vx, vy, omega));
 * </pre>
 */
public class Chassis extends SubsystemBase {

    ChassisConfig chassisConfig;
    public SwerveModule[] modules;
    private Pigeon gyro;

    private DemaciaKinematics demaciaKinematics;
    private SwerveDriveKinematics wpilibKinematics;
    private DemaciaPoseEstimator demaciaPoseEstimator;
    private Field2d field;

    public TagPose[] tags;
    private Field2d tagsField;
    public Quest quest;
    private Field2d questField;
    public ObjectPose objectPose;

    private StatusSignal<Angle> gyroYawStatus;
    // private TagPose limelight4;
    private TagPose limelight3;

    private Rotation2d lastGyroYaw;

    public Chassis(ChassisConfig chassisConfig) {
        setName(getName());

        this.chassisConfig = chassisConfig;

        modules = new SwerveModule[4];
        Translation2d[] modulePositions = new Translation2d[4];
        for (int i = 0; i < 4; i++) {
            modules[i] = new SwerveModule(chassisConfig.swerveModuleConfig[i]);
            modulePositions[i] = chassisConfig.swerveModuleConfig[i].position;
        }

        gyro = new Pigeon(chassisConfig.pigeonConfig);
        quest = new Quest();
        addStatus();
        demaciaKinematics = new DemaciaKinematics(modulePositions);
        wpilibKinematics = new SwerveDriveKinematics(modulePositions);
        demaciaPoseEstimator = new DemaciaPoseEstimator(
                modulePositions,
                new Matrix<>(VecBuilder.fill(0.02, 0.02, 0)),
                new Matrix<>(VecBuilder.fill(0.7, 0.7, Double.POSITIVE_INFINITY)));

        field = new Field2d();
        questField = new Field2d();
        tagsField = new Field2d();

        SmartDashboard.putData("chassis/reset gyro",
                new InstantCommand(() -> setYaw(Rotation2d.kZero)).ignoringDisable(true));
        SmartDashboard.putData("chassis/reset gyro 180",
                new InstantCommand(() -> setYaw(Rotation2d.kPi)).ignoringDisable(true));
        SmartDashboard.putData("chassis/field", field);
        SmartDashboard.putData("chassis/quest field", questField);
        SmartDashboard.putData("chassis/tags field", tagsField);
        SmartDashboard.putData("chassis/set coast",
                new InstantCommand(() -> setNeutralMode(false)).ignoringDisable(true));
        SmartDashboard.putData("chassis/set brake",
                new InstantCommand(() -> setNeutralMode(true)).ignoringDisable(true));
        SmartDashboard.putData("chassis", this);

        int c = 0;
        for (int i = 0; i < chassisConfig.tags.length; i++) {
            if (!chassisConfig.tags[i].getIsObjectCamera()) {
                c++;
            }
        }
        tags = new TagPose[c];
        int count = 0;
        for (int i = 0; i < chassisConfig.tags.length; i++) {
            if (!chassisConfig.tags[i].getIsObjectCamera()) {
                tags[count] = chassisConfig.tags[i];
                count++;
            }
        }
        RobotPose.initialize(modulePositions, new Matrix<>(
                new SimpleMatrix(
                        new double[] { 0.01, 0.01, 0 })),
                QUEST_STD);
    }

    private void addStatus() {
        gyroYawStatus = gyro.getYaw();
        lastGyroYaw = new Rotation2d(gyroYawStatus.getValueAsDouble());
    }

    public void setDrivePower(double pow, int id) {
        modules[id].setDrivePower(pow);
    }

    public void setDrivePower(double pow) {
        for (int i = 0; i < 4; i++)
            setDrivePower(pow, i);
    }

    /**
     * Checks all module electronics for faults and logs them.
     */
    public void checkElectronics() {
        for (SwerveModule module : modules) {
            module.checkElectronics();
        }
    }

    /**
     * Sets neutral mode (brake/coast) for all modules.
     * 
     * @param isBrake true for brake mode, false for coast
     */
    public void setNeutralMode(boolean isBrake) {
        for (SwerveModule module : modules) {
            module.setNeutralMode(isBrake);
        }
    }

    public void resetPose(Pose2d pose) {
        demaciaPoseEstimator.resetPose(pose);
    }

    /**
     * Gets the current estimated robot pose on the field.
     * 
     * @return Current pose (position and rotation) using odometry fusion
     */
    public Pose2d getPose() {
        return RobotPose.getInstance().getPose();
    }

    public Pose2d getPoseWithVelocity(double dt) {
        Pose2d currentPose = getPose();
        ChassisSpeeds currentSpeeds = getChassisSpeedsFieldRel();
        return new Pose2d(currentPose.getX() + (currentSpeeds.vxMetersPerSecond * dt),
                currentPose.getY() + (currentSpeeds.vyMetersPerSecond * dt),
                currentPose.getRotation().plus(new Rotation2d(currentSpeeds.omegaRadiansPerSecond * dt)));
    }

    private double targetAngle = 0;

    private boolean isRotateToHub = false;

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    public void setRotateToHub() {
        this.isRotateToHub = !isRotateToHub;
    }

    public Translation2d getDeliveryPoint() {
        Translation2d futurePos = getPoseWithVelocity(0.1).getTranslation();
        if (futurePos.getDistance(ShooterConstans.DELIVERY_POINT1) < futurePos
                .getDistance(ShooterConstans.DELIVERY_POINT2))
            return ShooterConstans.DELIVERY_POINT1;
        return ShooterConstans.DELIVERY_POINT2;
    }

    /**
     * Sets chassis velocities without acceleration limiting.
     * 
     * <p>
     * Applies discrete kinematics for accurate odometry.
     * Use this for precise path following where acceleration is pre-profiled.
     * </p>
     * 
     * @param speeds Desired chassis speeds (field-relative)
     */

    public void setVelocities(ChassisSpeeds speeds) {
        if (isRotateToHub) {
            speeds.omegaRadiansPerSecond = -1.5
                    * MathUtil.angleModulus(targetAngle - getPose().getRotation().getRadians());

        }

        SwerveModuleState[] states = demaciaKinematics
                .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroAngle()));
        // SwerveModuleState[] states = demaciaKinematics.toSwerveModuleStatesWithLimit(
        // speeds,
        // getChassisSpeedsFieldRel(),
        // getGyroAngle());
        setModuleStates(states);
    }

    public Translation2d getVelocityAsVector() {
        return new Translation2d(getChassisSpeedsFieldRel().vxMetersPerSecond,
                getChassisSpeedsFieldRel().vyMetersPerSecond);
    }

    /**
     * Sets robot-relative velocities with acceleration limiting.
     * 
     * <p>
     * Useful for manual control where joystick inputs are in robot frame.
     * </p>
     * <p>
     * Useful for manual control where joystick inputs are in robot frame.
     * </p>
     * 
     * @param speeds Desired chassis speeds (robot-relative)
     */
    public void setRobotRelSpeedsWithAccel(ChassisSpeeds speeds) {
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getGyroAngle());
        setVelocities(fieldSpeeds);
    }

    public void setSteerPositions(double[] positions) {
        for (int i = 0; i < positions.length; i++) {
            modules[i].setSteerPosition(positions[i]);
        }
    }

    public void setSteerPower(double pow, int id) {
        modules[id].setSteerPower(pow);
    }

    public double getSteerVelocity(int id) {
        return modules[id].getSteerVel();
    }

    public double getSteerAcceleration(int id) {
        return modules[id].getSteerAccel();
    }

    public void setSteerPositions(double position) {
        setSteerPositions(new double[] { position, position, position, position });
    }

    public ChassisSpeeds getRobotRelVelocities() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(getChassisSpeedsRobotRel(), getGyroAngle());
    }

    public void setRobotRelVelocities(ChassisSpeeds speeds) {
        SwerveModuleState[] states = wpilibKinematics.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }

    public void setDriveVelocities(double[] velocities) {
        for (int i = 0; i < velocities.length; i++) {
            modules[i].setDriveVelocity(velocities[i]);
        }
    }

    public void setDriveVelocities(double velocity) {
        setDriveVelocities(new double[] { velocity, velocity, velocity, velocity });
    }

    public Rotation2d getGyroAngle() {
        gyroYawStatus.refresh();
        if (gyroYawStatus.getStatus() == StatusCode.OK) {
            lastGyroYaw = new Rotation2d(gyroYawStatus.getValue());
        }
        return lastGyroYaw;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] arr = new SwerveModulePosition[modules.length];
        for (int i = 0; i < arr.length; i++) {
            arr[i] = modules[i].getModulePosition();
        }
        return arr;
    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            modules[i].setState(states[i]);
        }
    }

    private void updateVision(Pose2d pose) {
        demaciaPoseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp() - 0.05);
        demaciaPoseEstimator.setVisionMeasurementStdDevs(getSTD());
        tagsField.setRobotPose(pose);
    }

    // private void updateQuest(Pose2d questPose) {
    // demaciaPoseEstimator.addVisionMeasurement(questPose, Timer.getFPGATimestamp()
    // - 0.05);
    // demaciaPoseEstimator.setVisionMeasurementStdDevs(QUEST_STD);
    // questField.setRobotPose(questPose);
    // }

    // private Matrix<N3, N1> getSTD() {
    // double x = 0.05;
    // double y = 0.05;
    // double theta = 0.03;
    private void updateQuest(Pose2d questPose) {
        demaciaPoseEstimator.addVisionMeasurement(questPose, Timer.getFPGATimestamp() - 0.05);
        demaciaPoseEstimator.setVisionMeasurementStdDevs(QUEST_STD);
        questField.setRobotPose(questPose);
    }

    private Matrix<N3, N1> getSTD() {
        double x = 0.05;
        double y = 0.05;
        double theta = 0.03;

        // ChassisSpeeds currentSpeeds = getChassisSpeedsRobotRel();
        // double speed = Utilities.hypot(currentSpeeds.vxMetersPerSecond,
        // currentSpeeds.vyMetersPerSecond);

        // // Vision confidence adjustment
        // // if (visionFuse != null && visionFuse.getVisionConfidence() < 0.3) {
        // // x += 0.3;
        // // y += 0.3;
        // // }
        // Vision confidence adjustment
        // if (visionFuse != null && visionFuse.getVisionConfidence() < 0.3) {
        // x += 0.3;
        // y += 0.3;
        // }

        // // Speed-based confidence calculation
        // if (speed > WORST_RELIABLE_SPEED) {
        // // Maximum uncertainty for high speeds
        // x += 0.02;
        // y += 0.02;
        // } else if (speed <= BEST_RELIABLE_SPEED) {
        // // Minimum uncertainty for low speeds
        // x -= 0.02;
        // y -= 0.02;
        // } else {
        // // Calculate normalized speed for the falloff range
        // double normalizedSpeed = (speed - BEST_RELIABLE_SPEED)
        // / (WORST_RELIABLE_SPEED - BEST_RELIABLE_SPEED);

        // // Apply exponential falloff to calculate additional uncertainty
        // double speedConfidence = Math.exp(-3 * normalizedSpeed);

        // // Scale the uncertainty adjustment based on confidence
        // double adjustment = 0.02 * (1 - speedConfidence);
        // x += adjustment;
        // y += adjustment;
        // }

        return new Matrix<N3, N1>(new SimpleMatrix(new double[] { x, y, theta }));
    }

    Pose2d questPoseEstimation;

    Rotation2d gyroAngle;

    private boolean hasVisionUpdated = false;

    @Override
    public void periodic() {
        // visionFusePoseEstimation = visionFuse.getPoseEstimation();
        // gyroAngle = getGyroAngle();

        // OdometryObservation observation = new OdometryObservation(
        // Timer.getFPGATimestamp(),
        // getGyroAngle(),
        // getModulePositions());

        // demaciaPoseEstimator.addOdometryCalculation(observation,
        // getChassisSpeedsVector());
        // field.setRobotPose(getPose());

        // if (visionFusePoseEstimation != null) {
        // if (!hasVisionUpdated && quest.isConnected() && quest.isTracking()) {
        // hasVisionUpdated = true;
        // quest.setQuestPose(new Pose3d(new
        // Pose2d(visionFusePoseEstimation.getTranslation(), gyroAngle)));
        // }

        // updateVision(new Pose2d(visionFusePoseEstimation.getTranslation(),
        // gyroAngle));
        // visionFusePoseEstimation = null;
        // }
        // if (hasVisionUpdated && quest.isConnected() && quest.isTracking()) {
        // updateQuest(quest.getRobotPose2d());
        // }

        updateCommon();

        OdometryObservation observation = new OdometryObservation(
                Timer.getFPGATimestamp(),
                getGyroAngle(),
                getModulePositions());

        RobotPose.getInstance().update(observation, getVelocityAsVector());
        field.setRobotPose(getPose());
        field.getObject("estimation").setPose(
                ShooterUtils.computeFuturePosition(getChassisSpeedsFieldRel(), RobotCommon.currentRobotPose, 0.1));

    }

    public void updateCommon() {
        RobotCommon.currentRobotPose = getPose();
        RobotCommon.futureRobotPose = getFuturePose(0.04);
        RobotCommon.fieldRelativeSpeeds = getChassisSpeedsFieldRel();
        RobotCommon.robotRelativeSpeeds = getRobotRelVelocities();
        RobotCommon.robotAngle = getGyroAngle();
    }

    public Pose2d getFuturePose(double dtSeconds) {
        Pose2d poseAtTime = getPose().exp(new Twist2d(
                (getChassisSpeedsFieldRel().vxMetersPerSecond * dtSeconds),
                (getChassisSpeedsFieldRel().vyMetersPerSecond * dtSeconds),
                getChassisSpeedsFieldRel().omegaRadiansPerSecond * dtSeconds));
        return poseAtTime;
    }

    /**
     * Gets the current chassis speeds in robot-relative frame.
     * 
     * @return Current velocities in robot frame
     */
    public ChassisSpeeds getChassisSpeedsRobotRel() {
        return demaciaKinematics.toChassisSpeeds(
                getModuleStates(),
                Math.toRadians(gyroYawStatus.getValueAsDouble()));
    }

    /**
     * Gets the current chassis speeds in field-relative frame.
     * 
     * @return Current velocities transformed to field frame
     */
    public ChassisSpeeds getChassisSpeedsFieldRel() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                demaciaKinematics.toChassisSpeeds(getModuleStates(),
                        gyro.getAngularVelocityZWorld().getValue().in(RadiansPerSecond)),
                getGyroAngle());
    }

    public Translation2d getChassisSpeedsVector() {
        ChassisSpeeds s = getChassisSpeedsFieldRel();
        return new Translation2d(s.vxMetersPerSecond, s.vyMetersPerSecond);
    }

    /**
     * Returns the state of every module
     * 
     * 
     * @return Velocity in m/s, angle in Rotation2d
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] res = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            res[i] = modules[i].getState();
        }
        return res;
    }

    /**
     * Sets the gyro yaw angle (for field-relative reset).
     * 
     * <p>
     * Call this at the start of autonomous to set known field orientation.
     * </p>
     * 
     * @param angle New yaw angle (null to skip)
     */
    public void setYaw(Rotation2d angle) {
        if (angle != null) {
            gyro.setYaw(angle.getDegrees());
            quest.questResetfromRobotToQuest(angle);
            demaciaPoseEstimator
                    .resetPose(
                            new Pose2d(demaciaPoseEstimator.getEstimatedPose().getTranslation(), gyro.getRotation2d()));
        }
    }

    public ChassisConfig getConfig() {
        return chassisConfig;
    }

    /**
     * Stops all swerve modules immediately.
     */
    public void stop() {
        for (SwerveModule i : modules) {
            i.stop();
        }
    }
}
