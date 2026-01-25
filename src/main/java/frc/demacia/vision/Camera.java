// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */

public class Camera {

    private String name;
    private Translation3d robotToCamPosition;
    private double pitch;
    private double yaw;
    private String tableName;
    private boolean ishigher;// is higher than a tag 


    public Camera(String name, Translation3d robotToCamPosition, double pitch, double yaw, boolean ishigher) {
        this.name = name;
        this.robotToCamPosition = robotToCamPosition;
        this.pitch = pitch;
        this.yaw = yaw;
        this.ishigher = ishigher;

        this.tableName = "limelight-"+name;
    }
    public boolean getIsOnTurret(){
        return false;
    }
    public Supplier<Rotation2d> getTurrentAngle(){

        return () -> Rotation2d.kZero;
    }

    public Translation3d getRobotToCamPosition() {
        return !false ? robotToCamPosition : robotToCamPosition.rotateBy(Rotation3d.kZero);
    }

    public double getHeight() {
        return robotToCamPosition.getZ();
    }

    public double getPitch() {
        return this.pitch;
    }

    public double getYaw() {
        return this.yaw;
    }

    public String getName() {
        return this.name;
    }

    public String getTableName() {
        return this.tableName;
    }
    public boolean getIsHigher(){return this.ishigher;}
}