// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.command_factories.DumpFactory;
import frc.robot.util.Subsystems;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_buttonController = new CommandXboxController(1);

  private Subsystems m_subsystemContainer;

  public RobotContainer() {
    m_subsystemContainer = new Subsystems();
    configureDefaultCommands();
    configureBindings();
    configureNamedCommands();
  }

  private void configureDefaultCommands() {

  }

  private void configureBindings() {
    m_buttonController.a().whileTrue(DumpFactory.runRollers());
    m_buttonController.b().whileTrue(DumpFactory.runFloor());
  }

  private void configureNamedCommands() {

  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Nothing");
  }
}
