package frc.demacia.utils.geometry;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.units.measure.Distance;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** Game field object on a Field2d. */
public class FieldObject2dDemacia implements AutoCloseable {
  /**
   * Package-local constructor.
   *
   * @param name name
   */
  FieldObject2dDemacia(String name) {
    m_name = name;
  }

  @Override
  public void close() {
    if (m_entry != null) {
      m_entry.close();
    }
  }

  /**
   * Set the pose from a Pose object.
   *
   * @param pose 2D pose
   */
  public synchronized void setPose(Pose2d pose) {
    setPoses(pose);
  }

  /**
   * Set the pose from a Pose object.
   *
   * @param pose 2D pose
   */
  public synchronized void setPose(Pose2dDemacia pose) {
    setPoses(pose);
  }

  /**
   * Set the pose from x, y, and rotation.
   *
   * @param xMeters X location, in meters
   * @param yMeters Y location, in meters
   * @param rotation rotation
   */
  public synchronized void setPose(double xMeters, double yMeters, Rotation2d rotation) {
    setPose(new Pose2d(xMeters, yMeters, rotation));
  }

  /**
   * Set the pose from x, y, and rotation.
   *
   * @param xMeters X location, in meters 
   * @param yMeters Y location, in meters
   * @param rotation rotation
   */
  public synchronized void setPose(double xMeters, double yMeters, Rotation2dDemacia rotation) {
    setPose(new Pose2dDemacia(xMeters, yMeters, rotation));
  }

  /**
   * Set the pose from x, y, and rotation.
   *
   * @param x X location
   * @param y Y location
   * @param rotation rotation
   */
  public synchronized void setPose(Distance x, Distance y, Rotation2d rotation) {
    setPose(new Pose2d(x.in(Meters), y.in(Meters), rotation));
  }

  /**
   * Set the pose from x, y, and rotation.
   *
   * @param x X location
   * @param y Y location
   * @param rotation rotation
   */
  public synchronized void setPose(Distance x, Distance y, Rotation2dDemacia rotation) {
    setPose(new Pose2dDemacia(x.in(Meters), y.in(Meters), rotation));
  }

  /**
   * Get the pose.
   *
   * @return 2D pose
   */
  public synchronized Pose2dDemacia getPose() {
    updateFromEntry();
    if (!m_poses_demacia.isEmpty()) {
      return m_poses_demacia.get(0);
    }
    if (!m_poses.isEmpty()) {
      return new Pose2dDemacia(m_poses.get(0));
    }
    return new Pose2dDemacia(); // במקום Pose2d.kZero
  }

  /**
   * Set multiple poses from a list of Pose objects. The total number of poses is limited to 85.
   *
   * @param poses list of 2D poses
   */
  public synchronized void setPoses(List<Pose2d> poses) {
    m_poses.clear();
    m_poses_demacia.clear();
    m_poses.addAll(poses);
    updateEntry();
  }

  /**
   * Set multiple poses from a list of Pose objects. The total number of poses is limited to 85.
   *
   * @param poses list of 2D poses
   */
  public synchronized void setPosesDemacia(List<Pose2dDemacia> poses) {
    m_poses.clear();
    m_poses_demacia.clear();
    m_poses_demacia.addAll(poses);
    updateEntry();
  }

  /**
   * Set multiple poses from a list of Pose objects. The total number of poses is limited to 85.
   *
   * @param poses list of 2D poses
   */
  public synchronized void setPoses(Pose2d... poses) {
    m_poses.clear();
    m_poses_demacia.clear();
    Collections.addAll(m_poses, poses);
    updateEntry();
  }

  /**
   * Set multiple poses from a list of Pose objects. The total number of poses is limited to 85.
   *
   * @param poses list of 2D poses
   */
  public synchronized void setPoses(Pose2dDemacia... poses) {
    m_poses.clear();
    m_poses_demacia.clear();
    Collections.addAll(m_poses_demacia, poses);
    updateEntry();
  }

  /**
   * Sets poses from a trajectory.
   *
   * @param trajectory The trajectory from which the poses should be added.
   */
  public synchronized void setTrajectory(Trajectory trajectory) {
    m_poses.clear();
    m_poses_demacia.clear();
    for (Trajectory.State state : trajectory.getStates()) {
      m_poses.add(state.poseMeters); 
    }
    updateEntry();
  }

  /**
   * Get multiple poses.
   *
   * @return list of 2D poses
   */
  public synchronized List<Pose2dDemacia> getPoses() {
    updateFromEntry();
    List<Pose2dDemacia> allPoses = new ArrayList<>(m_poses_demacia);
    for (Pose2d pose : m_poses) {
      allPoses.add(new Pose2dDemacia(pose));
    }
    return allPoses;
  }

  void updateEntry() {
    updateEntry(false);
  }

  synchronized void updateEntry(boolean setDefault) {
    if (m_entry == null) {
      return;
    }

    double[] arr;
    int ndx = 0;

    if (!m_poses_demacia.isEmpty()) {
      arr = new double[m_poses_demacia.size() * 3];
      for (Pose2dDemacia pose : m_poses_demacia) {
        arr[ndx + 0] = pose.getX();
        arr[ndx + 1] = pose.getY();
        arr[ndx + 2] = pose.getRotation().getDegrees();
        ndx += 3;
      }
    } else {
      arr = new double[m_poses.size() * 3];
      for (Pose2d pose : m_poses) {
        Translation2d translation = pose.getTranslation();
        arr[ndx + 0] = translation.getX();
        arr[ndx + 1] = translation.getY();
        arr[ndx + 2] = pose.getRotation().getDegrees();
        ndx += 3;
      }
    }

    if (setDefault) {
      m_entry.setDefault(arr);
    } else {
      m_entry.set(arr);
    }
  }

  private synchronized void updateFromEntry() {
    if (m_entry == null) {
      return;
    }

    double[] arr = m_entry.get(null);
    if (arr != null) {
      if ((arr.length % 3) != 0) {
        return;
      }

      m_poses.clear();
      m_poses_demacia.clear();
      for (int i = 0; i < arr.length; i += 3) {
        m_poses.add(new Pose2d(arr[i], arr[i + 1], Rotation2d.fromDegrees(arr[i + 2])));
      }
    }
  }

  String m_name;
  DoubleArrayEntry m_entry;
  private final List<Pose2d> m_poses = new ArrayList<>();
  private final List<Pose2dDemacia> m_poses_demacia = new ArrayList<>();
}