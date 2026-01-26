package frc.demacia.vision.subsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.log.LogManager;
import frc.demacia.utils.log.LogEntryBuilder.LogLevel;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import static frc.demacia.vision.utils.VisionConstants.*;

public class Quest extends SubsystemBase {
  private Field2d field;
  private QuestNav questNav;
  private Pose3d currentQuestPose;
  private double timestamp;

    private Pose3d qPose3d = Pose3d.kZero;
  
  public Quest() {
    timestamp = 0;
    questNav = new QuestNav();
    questNav.commandPeriodic();

    field = new Field2d();
    currentQuestPose = new Pose3d(); // Initialize to origin - IMPORTANT!
    
    SmartDashboard.putData("Quest Field", field);
  }
  
  // Set robot pose (transforms to Quest frame and sends to QuestNav)
  public void setQuestPose(Pose3d currentBotpose){
    // currentQuestPose = currentBotpose.transformBy(ROBOT_TO_QUEST);

    questNav.setPose(currentBotpose.transformBy(ROBOT_TO_QUEST3D));// the transformBy is to switch x & y and gives back the hight of the quest

  }

 /**
  * * @return  the center of the robot form quest
  */
  public Pose2d getRobotPose2d() { 
    return currentQuestPose.toPose2d().transformBy(ROBOT_TO_QUEST2D);// the transformBy is to switch x & y
  }

  
  // Check if Quest is connected
  public boolean isConnected() {
    return questNav.isConnected();
  }

  // Check if Quest is tracking
  public boolean isTracking() {
    return questNav.isTracking();
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();

    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

    if (poseFrames.length > 0 && poseFrames[poseFrames.length - 1].isTracking()) {
      currentQuestPose = poseFrames[poseFrames.length - 1].questPose3d();
      timestamp = poseFrames[poseFrames.length - 1].dataTimestamp();
      // Display Quest pose

      // the quest x & y are oppeset so i am switching (if it were more than +-90 than i will had to transformBy)
      //never mind i will use transformby
      Pose2d qPose2d = currentQuestPose.toPose2d().transformBy(ROBOT_TO_QUEST2D);
      SmartDashboard.putNumber("Quest X", qPose2d.getX());
      SmartDashboard.putNumber("Quest Y", qPose2d.getY());
      SmartDashboard.putNumber("Quest Rotation", currentQuestPose.getRotation().getZ());

      field.setRobotPose(currentQuestPose.toPose2d());
    }
    
    // Diagnostics - helpful for debugging!
    SmartDashboard.putBoolean("Quest Connected", questNav.isConnected());
    SmartDashboard.putBoolean("Quest Tracking", questNav.isTracking());
    SmartDashboard.putNumber("Quest Latency (ms)", questNav.getLatency());
    SmartDashboard.putString("QuestNav Version", questNav.getLibVersion());
    
    // Battery monitoring
    questNav.getBatteryPercent().ifPresent(
        battery -> SmartDashboard.putNumber("Quest Battery %", battery));
  }

  // gives me the timestamp of the newst frame
  public double getTimestamp() {
    return timestamp;
  }

  public void questReset() {
    questNav.setPose(Pose3d.kZero);
  }
}