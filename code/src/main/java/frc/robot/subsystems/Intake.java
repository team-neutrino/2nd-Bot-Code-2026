
package frc.robot.subsystems;

import static frc.robot.util.Constants.IntakeConstants.*;
import static frc.robot.util.Constants.RioConstants.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonFX m_deployMotor;
  private TalonFX m_rollerMotor;
  private TalonFX m_rollerFollower;
  private TalonFXConfiguration m_deployMotorConfig;
  private TalonFXConfiguration m_rollerMotorConfig;
  private CurrentLimitsConfigs m_deployCurrentLimitsConfig;
  private CurrentLimitsConfigs m_rollerCurrentLimitsConfig;

  private PositionVoltage m_deployPositionControl;
  private VoltageOut m_rollerVoltageControl;

  private double m_rollerMotorVoltage;
  private double m_targetDeployAngle;

  private boolean m_isDeployed;

  public Intake() {
    m_deployMotor = new TalonFX(DEPLOY_MOTOR_ID, RIO_BUS);
    m_rollerMotor = new TalonFX(ROLLER_MOTOR_ID, RIO_BUS);
    m_rollerFollower = new TalonFX(ROLLER_FOLLOWER_ID, RIO_BUS);
    m_deployMotorConfig = new TalonFXConfiguration();
    m_rollerMotorConfig = new TalonFXConfiguration();
    m_deployCurrentLimitsConfig = new CurrentLimitsConfigs();
    m_rollerCurrentLimitsConfig = new CurrentLimitsConfigs();

    m_deployCurrentLimitsConfig.withSupplyCurrentLimit(DEPLOY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(DEPLOY_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);
    m_rollerCurrentLimitsConfig.withSupplyCurrentLimit(ROLLER_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(ROLLER_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);
    m_deployMotorConfig.CurrentLimits = m_deployCurrentLimitsConfig;
    m_rollerMotorConfig.CurrentLimits = m_rollerCurrentLimitsConfig;

    m_deployMotorConfig.Slot0.kP = INTAKE_kP;
    m_deployMotorConfig.Slot0.kI = INTAKE_kI;
    m_deployMotorConfig.Slot0.kD = INTAKE_kD;

    m_rollerMotor.getConfigurator().apply(m_rollerMotorConfig);
    m_rollerFollower.getConfigurator().apply(m_rollerMotorConfig);
    m_deployMotor.getConfigurator().apply(m_deployMotorConfig);
    m_rollerMotor.setNeutralMode(NeutralModeValue.Coast);
    m_rollerFollower.setNeutralMode(NeutralModeValue.Coast);
    m_deployMotor.setNeutralMode(NeutralModeValue.Coast);
    m_deployMotor.setPosition(0);

    m_deployPositionControl = new PositionVoltage(0);
    m_rollerVoltageControl = new VoltageOut(0);

    Follower followRequest = new Follower(ROLLER_MOTOR_ID, MotorAlignmentValue.Opposed);
    m_rollerFollower.setControl(followRequest);

    m_isDeployed = false;
  }

  public double getMotorAngle() {
    return m_deployMotor.getPosition().getValueAsDouble();
  }

  public double getTargetAngle() {
    return m_targetDeployAngle;
  }

  public double getRollerRPM() {
    return m_rollerMotor.getVelocity().getValueAsDouble() * 60;
  }

  public boolean isAtTarget() {
    return getMotorAngle() >= getTargetAngle() - ALLOWED_TARGET_ERROR
        && getMotorAngle() <= getTargetAngle() + ALLOWED_TARGET_ERROR;
  }

  private void moveToIntake(double targetPosition) {
    m_deployMotor.setControl(m_deployPositionControl.withPosition(targetPosition));
  }

  private void spinRoller(double voltage) {
    m_rollerVoltageControl.EnableFOC = true;
    m_rollerMotor.setControl(m_rollerVoltageControl.withOutput(voltage));
  }

  private void checkIsDeployed() {
    if (m_isDeployed) {
      m_targetDeployAngle = DEPLOYED_POSITION; // 0 should be changed when deploy angle is tested
    } else {
      m_targetDeployAngle = STARTING_POSITION;
    }
  }

  public void setIntakePID(double new_P, double new_I, double new_D) {
    m_deployMotorConfig.Slot0.kP = new_P;
    m_deployMotorConfig.Slot0.kI = new_I;
    m_deployMotorConfig.Slot0.kD = new_D;

    m_deployMotor.getConfigurator().apply(m_deployMotorConfig);
  }

  @Override
  public void periodic() {
    checkIsDeployed();
    spinRoller(m_rollerMotorVoltage);
    moveToIntake(m_targetDeployAngle);
  }

  public Command runIntake(double voltage) {
    return run(() -> {
      m_rollerMotorVoltage = voltage;
    });
  }

  public Command deployIntake() {
    return run(() -> {
      m_isDeployed = true;
    });
  }

  public Command retractIntake() {
    return run(() -> {
      m_isDeployed = false;
    });
  }

  public Command defaultCommand() {
    return run(() -> {
      m_rollerMotorVoltage = 0;
    });
  }
}
