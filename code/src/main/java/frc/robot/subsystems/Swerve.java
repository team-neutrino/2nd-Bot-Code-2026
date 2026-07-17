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
import frc.robot.util.Constants.GlobalConstants;
import frc.robot.generated.Telemetry;
import frc.robot.generated.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

import static frc.robot.util.Constants.SwerveConstants.*;

import java.io.IOException;

import org.json.simple.parser.ParseException;

public class Swerve extends CommandSwerveDrivetrain {
  private Telemetry m_telemetry = new Telemetry(MAX_SPEED);
  private RobotConfig m_robotConfig;
  private SlewRateLimiter m_slewLimit = new SlewRateLimiter(SLEW_LIMIT, -Integer.MAX_VALUE, 0);
  private double joystickVx;
  private double joystickVy;

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

  public Command defaultCommand(CommandXboxController joystick) {
        return run(() -> {
            double forward = -joystick.getLeftY();
            double left = -joystick.getLeftX();
            double rotation = -joystick.getRightX();
            double magnitude = Math.hypot(forward, left) * MAX_SPEED;
            magnitude = m_slewLimit.calculate(magnitude);
            joystickVx = forward * magnitude;
            joystickVy = left * magnitude;
            setControl(SwerveRequestStash.drive
                    .withVelocityY(joystickVy)
                    .withVelocityX(joystickVx)
                    .withRotationalRate(rotation * MAX_ROTATION_SPEED));
        });
  }

  public Command slowDriveCommand(CommandXboxController joystick) {
    return run(() -> {
        double forward = -joystick.getLeftY();
        double left = -joystick.getLeftX();
        double rotation = -joystick.getRightX();
        double magnitude = Math.hypot(forward, left) * SLOW_MAX_SPEED;
        magnitude = m_slewLimit.calculate(magnitude);
        joystickVx = forward * magnitude;
        joystickVy = left * magnitude;
        setControl(SwerveRequestStash.drive
                .withVelocityY(joystickVy)
                .withVelocityX(joystickVx)
                .withRotationalRate(rotation * SLOW_MAX_ROTATION_SPEED));
    });
  }

  public void setVelocity(double xVelocity, double yVelocity, Rotation2d targetDirection) {
    SwerveRequestStash.driveWithVelocity
        .withVelocityX(xVelocity)
        .withVelocityY(yVelocity)
        .withTargetDirection(targetDirection);
    setControl(SwerveRequestStash.driveWithVelocity);
  }

  public void configureRequestPID() {
    SwerveRequestStash.driveWithVelocity.HeadingController.setPID(SWERVE_P, SWERVE_I, SWERVE_D);
  }

  public class SwerveRequestStash {
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MAX_SPEED * 0.1).withRotationalDeadband(MAX_ROTATION_SPEED * 0.1)
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
