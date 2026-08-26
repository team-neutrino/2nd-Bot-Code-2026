package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.wpilibj.DriverStation;
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

  private int m_hubTagCount = 0;
  private double m_lastFrame = 0;

  private boolean m_enabled = false;

  private PoseEstimate m_estimateMT1;
  private PoseEstimate m_estimateMT2;

  private boolean m_poseZeroWasPublished = false;
  private boolean m_yawZeroWasPublished = false;
  private NetworkTableInstance m_nt = NetworkTableInstance.getDefault();
  private StructTopic<Pose2d> m_pose;
  private StructPublisher<Pose2d> m_posePub;
  private Pose2d blank = new Pose2d();
  private DoubleTopic m_yaw;
  private DoublePublisher m_yawPub;

  private Pose2d m_currentPose = new Pose2d();
  private Pose2d m_lastPose = new Pose2d();

  private boolean m_updatedImuModeSinceEnabled = false;

  private boolean m_captureRewindTriggered = false;

  private double m_last_update_timestamp = 0;

  public Vision() {
    m_timer.start();

    m_pose = m_nt.getStructTopic("/limelight_poses/" + LL, Pose2d.struct);
    m_yaw = m_nt.getDoubleTopic("/limelight_poses/yaw/" + LL + "Yaw");

    m_posePub = m_pose.publish();
    m_posePub.setDefault(blank);

    m_yawPub = m_yaw.publish(PubSubOption.keepDuplicates(false));

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

  private double getFrame() {
    return NetworkTableInstance.getDefault() // presumably using a default table that limelight publishes to
        .getTable(LL)
        .getEntry("hb")
        .getDouble(-1);
  }

  // makes sure megatag 1 pose is Real....
  private boolean verifyMT1() {
    return m_estimateMT1 != null &&
        m_estimateMT1.tagCount != 0 &&
        !m_estimateMT1.isMegaTag2 &&
        !Double.isNaN(m_estimateMT1.avgTagDist);
  }

  // makes sure megatag 2 pose is Real....
  private boolean verifyMT2() {
    return m_estimateMT2 != null
        && m_estimateMT2.tagCount != 0
        && m_estimateMT2.isMegaTag2 // I would sure hope so
        && !Double.isNaN(m_estimateMT2.avgTagDist)
        && poseInField(m_estimateMT2);
  }

  /*
   * Let's not accept any pose that's outside of the field boundaries
   */
  private boolean poseInField(PoseEstimate poseEstimate) {
    if (poseEstimate == null || poseEstimate.pose.getTranslation().equals(Translation2d.kZero)) {
      return false;
    }

    return poseEstimate.pose.getX() > 0
        && poseEstimate.pose.getX() < FIELD_DIMENSION_X
        && poseEstimate.pose.getY() > 0
        && poseEstimate.pose.getY() < FIELD_DIMENSION_Y;
  }

  /*
   * Limelight has two MegaTag algorithms that are used to verify robot position.
   * MegaTag 1 is suitable only for determining robot yaw, while MegaTag 2 is
   * suitable for determining robot pose
   * This code takes the best estimate from MT1 and MT2, fuses them if applicable,
   * and then feeds that into swerve
   */
  private void updateFusionMT() {
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
          m_estimateMT1.pose.getRotation());
      timestamp = m_estimateMT2.timestampSeconds;
    }

    m_currentPose = pose;

    if (m_currentPose.equals(m_lastPose)) {
      return;
    }

    if (m_last_update_timestamp > timestamp) {
      m_last_update_timestamp = timestamp;
      swerve.addVisionMeasurement(pose, timestamp,
          VecBuilder.fill(getCalcXYStdev(), getCalcXYStdev(), getCalcYawStdev()));
    }
  }

  // give the pigeon (thing that tells us our rotation) information from vision if
  // our mt1 data is good
  private boolean verifyPigeonSeedUpdate() {
    return m_estimateMT1 != null // don't crash
        && ((m_estimateMT1.tagCount > 1) || (m_estimateMT1.tagCount == 1 && !m_enabled)) // only if mt1 sees more than
                                                                                         // one tag or IDK
        && Math.abs(swerve.getState().Speeds.omegaRadiansPerSecond) < Math.PI / 4
        && poseInField(m_estimateMT1) // idk
        && swerve.getSpeedMetersPerSecond() < PIGEON_SEED_XY_THRESHOLD
        && m_timer.hasElapsed(PIGEON_SEED_PERIOD) // don't reset pigeon too often
        && m_estimateMT1.avgTagDist < PIGEON_SEED_DISTANCE_THRESHOLD; // only reset pigeon if the tags are far enough
                                                                      // apart to give good data
  }

  public void updatePigeonSeed() {
    if (verifyPigeonSeedUpdate() && !m_currentPose.equals(m_lastPose)) {
      swerve.seedYawMT1(m_estimateMT1.pose.getRotation().getDegrees(),
          MT1_WEIGHT_YAW);
      m_timer.restart();
    }
  }

  public double getEstimateYawMT1() {
    if (m_estimateMT1 == null) {
      return IGNORE_MEASUREMENT_STD_DEV;
    }
    return m_estimateMT1.pose.getRotation().getDegrees();
  }

  public Pose2d getEstimatePose() {
    if (m_estimateMT2 == null) {
      return Pose2d.kZero;
    }
    return m_estimateMT2.pose;
  }

  public void publishPose() {
    if (!m_poseZeroWasPublished && getEstimatePose().equals(Pose2d.kZero)) {
      m_posePub.set(getEstimatePose());
      m_poseZeroWasPublished = true;
    } else if (!getEstimatePose().equals(Pose2d.kZero)) {
      m_posePub.set(getEstimatePose());
      m_poseZeroWasPublished = false;
    }
  }

  public void publishYaw() {
    if (!m_yawZeroWasPublished && getEstimateYawMT1() == IGNORE_MEASUREMENT_STD_DEV) {
      m_yawPub.set(getEstimateYawMT1());
      m_yawZeroWasPublished = true;
    } else if (getEstimateYawMT1() != IGNORE_MEASUREMENT_STD_DEV) {
      m_yawPub.set(getEstimateYawMT1());
      m_yawZeroWasPublished = false;
    }
  }

  public void publishDefaultPose() {
    if (!m_poseZeroWasPublished) {
      m_posePub.set(Pose2d.kZero);
      m_poseZeroWasPublished = true;
    }
  }

  public void publishDefaultYaw() {
    if (!m_yawZeroWasPublished) {
      m_yawPub.set(IGNORE_MEASUREMENT_STD_DEV);
      m_yawZeroWasPublished = true;
    }
  }

  public double getErrorFactor() {
    double errorFactor = 0.05;

    // some code here if it is relevant

    return errorFactor;
  }

  private double getMinimumStdDev() {
    double minStdDev = 0.1;

    // some code here if it is relevant

    return minStdDev;
  }

  private double getMinimumStdDevTheta() {
    double minStdDev = 4;

    // some code here if it is relevant

    return minStdDev;
  }

  /*
   * This calculates XY standard deviation, which is how confident the vision
   * system is in its estimate of our position laterally.
   */
  private double setXYstdev(double distance, double numberOfTags, int numberOfHubTags) {
    double minimumXyStdDev = getMinimumStdDev();
    // if (onBump()) {
    // return minimumXyStdDev;
    // }

    double errorFactor = getErrorFactor();

    if (numberOfHubTags < 2) {
      errorFactor *= 10.0;
      minimumXyStdDev *= 10.0;
    }

    return Math.max(
        minimumXyStdDev,
        (Math.pow(distance, 2) * errorFactor) / Math.pow(numberOfTags, 2));
  }

  /** Computes rotational standard deviation using tag distance. */
  private double setThetastdev(double distance, int numberOfHubTags) {
    if (!verifyMT1()) {
      return IGNORE_MEASUREMENT_STD_DEV;
    }
    double errorFactor = getErrorFactor();
    double minimumThetaStDev = getMinimumStdDevTheta();
    if (numberOfHubTags < 2) {
      errorFactor *= 10.0;
      minimumThetaStDev *= 10.0;
    }

    return Math.max(minimumThetaStDev, (Math.pow(distance, 2) * errorFactor));
  }

  private double getCalcYawStdev() {
    if (!verifyMT1()) {
      return IGNORE_MEASUREMENT_STD_DEV;
    }
    double distance = m_estimateMT1.avgTagDist;
    return setThetastdev(distance, m_hubTagCount);
  }

  /** Calculates XY measurement standard deviation dynamically. */
  private double getCalcXYStdev() {
    if (!verifyMT2()) {
      return IGNORE_MEASUREMENT_STD_DEV;
    }

    double numberOfTags = m_estimateMT2.tagCount;
    double distance = m_estimateMT2.avgTagDist;

    return setXYstdev(distance, numberOfTags, m_hubTagCount);
  }

  private void updateHubTagCount(PoseEstimate estimate) {
    if (estimate == null) {
      m_hubTagCount = 0;
      return;
    }

    int count = 0;

    for (RawFiducial fiducial : estimate.rawFiducials) { // we need to iterate through all the possible tags we see to
                                                         // check if any of them are hub tags
      if (ALL_HUB_TAGS.contains(fiducial.id)) {
        count += 1;
      }
    }

    m_hubTagCount = count;
  }

  /** Supplies robot orientation to the Limelight for IMU fusion. */
  public void setRobotOrientation(double yawDeg, double yawRate, double pitchDeg,
      double pitchRate, double rollDeg, double rollRate) {
    LimelightHelpers.SetRobotOrientation(LL, yawDeg, yawRate, pitchDeg, pitchRate, rollDeg, rollRate);

  }

  /** Adjusts IMU fusion mode dynamically based on enable state. */

  // 0 EXTERNAL_ONLY External (NT/HTTP) No internal IMU processing. MT2 uses
  // interpolated yaw from robot's gyro sent via SetRobotOrientation().
  // 1 EXTERNAL_SEED External (NT/HTTP) Internal IMU offset is calibrated to match
  // external yaw each frame (seeding). MT2 still uses external yaw for botpose.
  // 2 INTERNAL_ONLY Internal IMU Uses internal IMU's fused yaw only. No external
  // input required.
  // 3 INTERNAL_MT1_ASSIST Internal IMU + MT1 Complementary filter fuses internal
  // IMU with MT1 vision yaw. When MT1 gets a valid pose, it slowly corrects
  // internal IMU drift.
  // 4 INTERNAL_EXTERNAL_ASSIST Internal IMU + External IMU Complementary filter
  // fuses internal IMU with external yaw from SetRobotOrientation(). This is the
  // recommended mode, as the internal IMU's 1khz update rate is utilized for
  // frame-by-frame motion while the robot's IMU corrects for any drift over time.

  // copy and pasted from hal's code : )

  private void setIMUMode() {
    if (!m_enabled) {
      LimelightHelpers.SetIMUMode(LL, 0);
      m_updatedImuModeSinceEnabled = false;
    } else {
      LimelightHelpers.SetIMUMode(LL, 3);
    }
  }

  public void triggerCaptureRewind() {
    if (DriverStation.getMatchTime() <= 1.0 && !m_captureRewindTriggered) {
      LimelightHelpers.triggerRewindCapture(LL, 165);
      m_captureRewindTriggered = true;
    }
  }

  public boolean hasTwoHubTags() {
    return m_hubTagCount > 1;
  }

  @Override
  public void periodic() {
    if (swerve == null) {
      return;
    }

    final double yaw_degrees = swerve.getYawDegrees();
    final double pitch_degrees = swerve.getPitch();
    final double roll_degrees = swerve.getRoll();
    final double yaw_rate = swerve.getYawRate();
    final double pitch_rate = swerve.getPitchRate();
    final double roll_rate = swerve.getRollRate();
    setRobotOrientation(yaw_degrees, yaw_rate, pitch_degrees, pitch_rate,
        roll_degrees, roll_rate);

    m_enabled = DriverStation.isEnabled();

    setIMUMode();
    triggerCaptureRewind();

    updateFusionMT();
    updatePigeonSeed();
    updateHubTagCount(m_estimateMT2);
    m_lastPose = m_currentPose;
  }
}
