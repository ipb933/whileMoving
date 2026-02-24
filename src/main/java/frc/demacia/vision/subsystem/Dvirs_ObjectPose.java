// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.vision.subsystem;



import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.demacia.vision.Camera;
import frc.robot.RobotCommon;

/** Add your docs here. */
public class Dvirs_ObjectPose {

    private NetworkTable Table;

    private Camera objectCam;

    private double camObjectYaw;
    private double camObjectPitch;
    private double dist;

    private Translation2d cameraToObject;
    private Translation2d robotToObject;
    private Translation2d fieldToObject;

    private double tempsize =0;

    

    public Dvirs_ObjectPose(Camera objectCam){
        this.objectCam = objectCam;
        Table = NetworkTableInstance.getDefault().getTable(objectCam.getTableName());

    }

    public Translation2d giveBestTranslation(){
        if(isObjectDetected()){
            updateValues();
            getDistance();
            getRobotToObjectFeildRel();
            return getOriginToObject();
        }
        return Translation2d.kZero;
    }

    public boolean isObjectDetected(){
        return Table.getEntry("tv").getDouble(0.0) != 0;
    }

    public void updateValues(){
        camObjectPitch = Table.getEntry("ty").getDouble(0.0);
        camObjectYaw = (-Table.getEntry("tx").getDouble(0.0));
    }

    public double getDistance(){
        double alpha = Math.abs(camObjectPitch + objectCam.getPitch());
        dist = (Math.abs(objectCam.getHeight())/ (Math.tan(Math.toRadians(alpha))));
        return dist;
    }

    public Translation2d getRobotToObjectFeildRel(){
        cameraToObject = new Translation2d(getDistance(),
            Rotation2d.fromDegrees(camObjectYaw + objectCam.getYaw()));
        robotToObject = (objectCam.getRobotToTurretPosition().toTranslation2d().plus(cameraToObject))
            .rotateBy(RobotCommon.robotAngle);
        return robotToObject;
    }
    
    public Translation2d getOriginToObject(){
        
        if (robotToObject != null) {

            fieldToObject = RobotCommon.currentRobotPose.getTranslation().minus(getRobotToObjectFeildRel());

            return fieldToObject;
        }
        return new Translation2d();
    }
}
