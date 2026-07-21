package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonFX m_deployMotor;
  private TalonFX m_rollerMotor;
  private TalonFX m_rollerFollower;

  public Intake() {

  }

  @Override
  public void periodic() {
  }
}
