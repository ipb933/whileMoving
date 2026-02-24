// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.odometry;

import static frc.demacia.vision.utils.VisionConstants.BEST_RELIABLE_SPEED;
import static frc.demacia.vision.utils.VisionConstants.WORST_RELIABLE_SPEED;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.demacia.odometry.DemaciaPoseEstimator.OdometryObservation;
import frc.demacia.utils.Utilities;
import frc.demacia.utils.log.LogManager;
import frc.demacia.vision.subsystem.Quest;
import frc.demacia.vision.utils.Vision;
import frc.demacia.vision.utils.VisionConstants;
import frc.robot.RobotCommon;
import frc.robot.turret.subsystems.Turret;

/** Add your docs here. */
public class RobotPose {
    private static RobotPose instance;

    private Vision vision;
    private DemaciaPoseEstimator poseEstimator;
    private Quest quest;

    private Matrix<N3, N1> questSTD;

    private boolean hasVisionUpdated;

    private Matrix<N3, N1> visionSTD;

    private RobotPose(Translation2d[] modulePositions, Matrix<N3, N1> stateSTD,
            Matrix<N3, N1> questSTD) {
        this.vision = new Vision((VisionConstants.Tags.TAGS_ARRAY));

        this.quest = new Quest();
        this.questSTD = questSTD;
        this.visionSTD = new Matrix<N3, N1>(new SimpleMatrix(new double[] { 0.1, 0.1, 999999 }));
        this.hasVisionUpdated = false;
        this.poseEstimator = new DemaciaPoseEstimator(modulePositions, stateSTD, visionSTD);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPose();
    }

    public static void initialize(Translation2d[] modulePositions, Matrix<N3, N1> stateSTD,
            Matrix<N3, N1> questSTD) {

        if (instance == null)
            instance = new RobotPose(modulePositions, stateSTD, questSTD);
    }

    public static RobotPose getInstance() {
        return instance;
    }

    public void addOdometryCalculation(OdometryObservation odometryObservation, Translation2d currentVelocity) {
        poseEstimator.addOdometryCalculation(odometryObservation, currentVelocity);
    }

    public void addOdometryCalculation(Pose2d odometryPose, Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions, Translation2d currentVelocity) {
        addOdometryCalculation(new OdometryObservation(Timer.getFPGATimestamp(), gyroAngle, modulePositions),
                currentVelocity);
    }

    public void addVisionMeasurement() {
        vision.updateValues();

        Pose2d visionPose = vision.getPoseEstimation();
        double timestamp = Timer.getFPGATimestamp() - 0.05;
        if (!hasVisionUpdated) {
            quest.setQuestPose(new Pose3d(visionPose));
            hasVisionUpdated = true;
        }

        poseEstimator.setVisionMeasurementStdDevs(getSTD());
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
    }

    public void addQuestMeasurement() {

        Pose2d questPose = quest.getRobotPose2d();
        double timestamp = Timer.getFPGATimestamp() - 0.05;

        poseEstimator.setVisionMeasurementStdDevs(questSTD);
        poseEstimator.addVisionMeasurement(questPose, timestamp);

    }

    public void update(Pose2d odometryPose, Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions, Translation2d currentVelocity) {
        update(new OdometryObservation(Timer.getFPGATimestamp(), gyroAngle, modulePositions), currentVelocity);

    }

    private boolean shouldUpdateVision() {
        // return (Math.hypot(RobotCommon.fieldRelativeSpeeds.vxMetersPerSecond,
        //         RobotCommon.fieldRelativeSpeeds.vyMetersPerSecond) <= 3
        //         // && Turret.getInstance().getTurretVelocity() <= Math.toRadians(100)
        //         && vision.isSeeTagWithDistance());

        return vision.isSeeTag();

    }

    public void update(OdometryObservation odometryObservation, Translation2d currentVelocity) {
        addOdometryCalculation(odometryObservation, currentVelocity);

        if (hasVisionUpdated && quest.isConnected())
            addQuestMeasurement();
        if (shouldUpdateVision()) {
            addVisionMeasurement();
        } 
    }

    private static Matrix<N3, N1> getSTD() {
        double x = 0.05;
        double y = 0.05;
        double theta = 0.03;

        ChassisSpeeds currentSpeeds = RobotCommon.robotRelativeSpeeds;
        double speed = Utilities.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        // Vision confidence adjustment
        // if (visionFuse != null && visionFuse.getVisionConfidence() < 0.3) {
        // x += 0.3;
        // y += 0.3;
        // }

        // Speed-based confidence calculation
        if (speed > WORST_RELIABLE_SPEED) {
            // Maximum uncertainty for high speeds
            x += 0.02;
            y += 0.02;
        } else if (speed <= BEST_RELIABLE_SPEED) {
            // Minimum uncertainty for low speeds
            x -= 0.02;
            y -= 0.02;
        } else {
            // Calculate normalized speed for the falloff range
            double normalizedSpeed = (speed - BEST_RELIABLE_SPEED)
                    / (WORST_RELIABLE_SPEED - BEST_RELIABLE_SPEED);

            // Apply exponential falloff to calculate additional uncertainty
            double speedConfidence = Math.exp(-3 * normalizedSpeed);

            // Scale the uncertainty adjustment based on confidence
            double adjustment = 0.02 * (1 - speedConfidence);
            x += adjustment;
            y += adjustment;
        }

        return new Matrix<N3, N1>(new SimpleMatrix(new double[] { x, y, theta }));
    }

}
