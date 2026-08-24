package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

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

  }

  @Override
  public void periodic() {
  }
}
