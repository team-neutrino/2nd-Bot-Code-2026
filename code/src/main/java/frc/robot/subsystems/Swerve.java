// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Telemetry;
import frc.robot.util.generated.CommandSwerveDrivetrain;
import frc.robot.util.generated.TunerConstants;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;

import static frc.robot.Constants.SwerveConstants.*;

import java.io.IOException;

import org.json.simple.parser.ParseException;

public class Swerve extends CommandSwerveDrivetrain {
  private Telemetry m_telemetry = new Telemetry(MAX_SPEED);
  private RobotConfig m_robotConfig;

  public Swerve() {
    super(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight,
        TunerConstants.BackLeft, TunerConstants.BackRight);
    resetRotation(Rotation2d.fromDegrees(getYawDegrees()));
    configureRequestPID();
    registerTelemetry(m_telemetry::telemeterize);
    configurePathPlanner();
  }

  private void configurePathPlanner() {
    double pTranslation = 1;
    double iTranslation = 0;
    double dTranslation = 0;
    double pRotation = 1;
    double iRotation = 0;
    double dRotation = 0;
    PIDConstants translationConstants = new PIDConstants(pTranslation, iTranslation, dTranslation);
    PIDConstants rotationConstants = new PIDConstants(pRotation, iRotation, dRotation);

    try {
      m_robotConfig = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getCurrentPose,
          this::resetPose,
          this::getChassisSpeeds,
          this::setControlAndApplyChassis,
          new PPHolonomicDriveController(
              translationConstants,
              rotationConstants),
          m_robotConfig,
          () -> {
            return GlobalConstants.RED_ALLIANCE.isPresent() && GlobalConstants.RED_ALLIANCE.get();
          },
          this);
    } catch (IOException | ParseException e) {
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }
  }

  public double getYaw360() {
    return getPigeon2().getYaw().getValueAsDouble() % 360;
  }

  public double getYawDegrees() {
    return Math.toDegrees(getYawRadians());
  }

  public double getYawRadians() {
    return MathUtil.angleModulus(Math.toRadians(getPigeon2().getYaw().getValueAsDouble()));
  }

  public void resetYaw() {
    resetRotation(new Rotation2d(0));
    getPigeon2().reset();
  }

  public Command resetYawCommand() {
    return run(() -> resetYaw());
  }

  public Pose2d getCurrentPose() {
    return getState().Pose;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return getState().Speeds;
  }

  public void setControlAndApplyChassis(ChassisSpeeds speeds) {
    setControl(
        SwerveRequestStash.autonDrive.withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond));
  }

  public Command defaultCommand(CommandXboxController driverController) {
    return applyRequest(() -> SwerveRequestStash.drive.withVelocityX(-driverController.getLeftY() * MAX_SPEED)
        .withVelocityY(-driverController.getLeftX() * MAX_SPEED)
        .withRotationalRate(-driverController.getRightX() * MAX_ANGULAR_RATE));
  }

  public Command slowDriveCommand(CommandXboxController driverController) {
    return applyRequest(() -> SwerveRequestStash.drive.withVelocityX(-driverController.getLeftY() * SLOW_SPEED)
        .withVelocityY(-driverController.getLeftX() * SLOW_SPEED)
        .withRotationalRate(-driverController.getRightX() * SLOW_ANGULAR_RATE));
  }

  public void setVelocity(double xVelocity, double yVelocity, Rotation2d targetDirection) {
    SwerveRequestStash.driveWithVelocity
        .withVelocityX(xVelocity)
        .withVelocityY(yVelocity)
        .withTargetDirection(targetDirection);
    setControl(SwerveRequestStash.driveWithVelocity);
  }

  public void configureRequestPID() {
    SwerveRequestStash.driveWithVelocity.HeadingController.setPID(DRIVE_TO_POINT_P, DRIVE_TO_POINT_I, DRIVE_TO_POINT_D);
  }

  public class SwerveRequestStash {
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MAX_SPEED * 0.1).withRotationalDeadband(MAX_ANGULAR_RATE * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public static final SwerveRequest.FieldCentricFacingAngle driveWithVelocity = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.Velocity).withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
    public static final SwerveRequest.RobotCentric autonDrive = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.Velocity);
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
