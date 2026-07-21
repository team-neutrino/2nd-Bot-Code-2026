package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;

import static frc.robot.util.Constants.LimelightConstants.*;
import static frc.robot.util.Constants.FieldMeasurementConstants.*;
import static frc.robot.util.Constants.AprilTagConstants.*;
import static frc.robot.util.Subsystems.swerve;

public class Vision extends SubsystemBase {
  private Timer m_timer = new Timer();

  private double m_hubTagCount = 0;
  private double m_lastFrame = 0;

  private PoseEstimate m_estimateMT1;
  private PoseEstimate m_estimateMT2;

  private Pose2d m_currentPose = new Pose2d();
  private Pose2d m_lastPose = new Pose2d();

  private double m_last_update_timestamp = 0;

  public Vision() {
    m_timer.start();

    LimelightHelpers.setLEDMode_ForceOff("limelight"); // no more blinding me
    LimelightHelpers.setCameraPose_RobotSpace(LL,
      LL_FORWARD_OFFSET, 
      LL_SIDE_OFFSET, 
      LL_FORWARD_OFFSET, 
      LL_ROLL_OFFSET, 
      LL_PITCH_OFFSET, 
      LL_YAW_OFFSET);
    LimelightHelpers.SetFiducialDownscalingOverride(LL, 1); // Do YOU know what this does ? I don't
    
    LimelightHelpers.setPipelineIndex(LL, 0);
    
    if (LL_MODEL == 4) {
       LimelightHelpers.setRewindEnabled(LL, true);
       LimelightHelpers.SetIMUAssistAlpha(LL, EXTERNAL_WEIGHT);
    }
  }

  public double getFrame() {
    return NetworkTableInstance.getDefault() // presumably using a default table that limelight publishes to
              .getTable(LL)
              .getEntry("hb")
              .getDouble(-1);
  }

  // makes sure megatag 1 pose is Real....
  public boolean verifyMT1() {
    return m_estimateMT1 != null &&
    m_estimateMT1.tagCount != 0 &&
    !m_estimateMT1.isMegaTag2 &&
    !Double.isNaN(m_estimateMT1.avgTagDist);
  }

  // makes sure megatag 2 pose is Real....
  public boolean verifyMT2() {
    return m_estimateMT2 != null
     && m_estimateMT2.tagCount != 0
     && m_estimateMT2.isMegaTag2 // I would sure hope so
     && !Double.isNaN(m_estimateMT2.avgTagDist)
     && poseInField(m_estimateMT2);
  }

  /*
    Let's not accept any pose that's outside of the field boundaries
   */
  public boolean poseInField(PoseEstimate poseEstimate) {
    if (poseEstimate == null || poseEstimate.pose.getTranslation().equals(Translation2d.kZero)) {
      return false;
    }

    return poseEstimate.pose.getX() > 0
    && poseEstimate.pose.getX() < FIELD_DIMENSION_X
    && poseEstimate.pose.getY() > 0
    && poseEstimate.pose.getY() < FIELD_DIMENSION_Y;
  }

  /*
    Limelight has two MegaTag algorithms that are used to verify robot position.
    MegaTag 1 is suitable only for determining robot yaw, while MegaTag 2 is suitable for determining robot pose
    This code takes the best estimate from MT1 and MT2, fuses them if applicable, and then feeds that into swerve
  */
  public void updateFusionMT() {
    final double frame = getFrame();

    if (frame <= m_lastFrame || frame < 0.0) { // don't use any frames from the past or that we've alr used
      m_hubTagCount = 0;
      return;
    }

    m_estimateMT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(LL);
    m_estimateMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LL);

    double timestamp;
    Pose2d pose;

    if (!verifyMT2()) { // any view with an invalid MT2 pose is useless for this algorithm
      return;
    }

    if (!verifyMT1()) {
      pose = m_estimateMT2.pose;
      timestamp = m_estimateMT2.timestampSeconds;
    } else {
      pose = new Pose2d( // combine pose from MT2 and yaw from MT1
        m_estimateMT2.pose.getTranslation(),
        m_estimateMT1.pose.getRotation()
      );
      timestamp = m_estimateMT2.timestampSeconds;
    }

    m_currentPose = pose;

    if (m_currentPose.equals(m_lastPose)) {
      return;
    }

    if (m_last_update_timestamp > timestamp) {
      m_last_update_timestamp = timestamp;
      System.out.println("Yay");
    }
  }

  public void updateHubTagCount(PoseEstimate estimate) {
    if (estimate == null) {
      m_hubTagCount = 0;
      return;
    }

    int count = 0;

    for (RawFiducial fiducial : estimate.rawFiducials) {
      if (ALL_HUB_TAGS.contains(fiducial.id)) {
        count += 1;
      }
    }

    m_hubTagCount = count;
  }

  public boolean hasTwoHubTags() {
    return m_hubTagCount > 1;
  }
  
  @Override
  public void periodic() {
    if (swerve == null) {
      return;
    }

    updateFusionMT();
    updateHubTagCount(m_estimateMT2);
    m_lastPose = m_currentPose;
  }
}
