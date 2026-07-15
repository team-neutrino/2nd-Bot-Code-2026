package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import static frc.robot.util.Constants.RioConstants.*;
import static frc.robot.util.Constants.DumpConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dump extends SubsystemBase {
  private TalonFX m_leftRoller;
  private TalonFX m_leftRollerFollow;
  private TalonFX m_rightRoller;
  private TalonFX m_rightRollerFollow;
  private TalonFX m_floor;
  private TalonFX m_floorFollow;

  private TalonFXConfiguration m_leftRollerConfig;
  private TalonFXConfiguration m_rightRollerConfiguration;
  private TalonFXConfiguration m_floorConfig;

  private CurrentLimitsConfigs m_rollerCurrentConfig;
  private CurrentLimitsConfigs m_floorCurrentConfig;

  public Dump() {
    m_leftRoller = new TalonFX(LEFT_ROLLER_ID, RIO_BUS);
    m_leftRollerFollow = new TalonFX(LEFT_ROLLER_FOLLOWER_ID, RIO_BUS);
    m_rightRoller = new TalonFX(RIGHT_ROLLER_ID, RIO_BUS);
    m_rightRollerFollow = new TalonFX(RIGHT_ROLLER_FOLLOWER_ID, RIO_BUS);
    m_floor = new TalonFX(FLOOR_ID, RIO_BUS);
    m_floorFollow = new TalonFX(FLOOR_FOLLOWER_ID, RIO_BUS);

    m_rollerCurrentConfig.withSupplyCurrentLimit(ROLLER_CURRENT_LIMIT).withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(ROLLER_CURRENT_LIMIT).withStatorCurrentLimitEnable(true);

    m_leftRollerConfig.Slot0.kP = ROLLER_KP;
    m_leftRollerConfig.Slot0.kI = ROLLER_KI;
    m_leftRollerConfig.Slot0.kD = ROLLER_KD;

    m_rightRollerConfiguration = m_leftRollerConfig;
  }

  @Override
  public void periodic() {
  }
}
