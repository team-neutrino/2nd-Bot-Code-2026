package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;

import static frc.robot.util.Constants.LimelightConstants.*;
import static frc.robot.util.Subsystems.swerve;

public class Vision extends SubsystemBase {
  private Timer m_timer = new Timer();

  private double m_hubTagCount = 0;
  private double m_lastFrame = 0;

  private PoseEstimate m_estimateMT1;
  private PoseEstimate m_estimateMT2;

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

  public void updateFusionMT() {
    final double frame = getFrame();

    if (frame <= m_lastFrame || frame < 0.0) { // don't use any frames from the past or that we've alr used
      m_hubTagCount = 0;
      return;
    }

    m_estimateMT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(LL);
    m_estimateMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LL);

  }

  @Override
  public void periodic() {
    if (swerve == null) {
      return;
    }



  }
}
