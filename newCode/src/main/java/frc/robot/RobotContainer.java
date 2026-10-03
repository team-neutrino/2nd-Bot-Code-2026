
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.command_factories.DumpFactory;
import frc.robot.command_factories.IntakeFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.Subsystems;
import static frc.robot.util.Subsystems.*;

public class RobotContainer {
    private Subsystems m_subsystemContainer;

    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_buttonController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        m_subsystemContainer = new Subsystems();
        configureBindings();
        setDefaultCommands();
    }

    private void configureBindings() {
        m_driverController.back().whileTrue(swerve.resetYaw());
        m_driverController.povLeft().whileTrue(swerve.slowSwerveDrive(m_driverController));
        m_driverController.povRight().whileTrue(swerve.slowestSwerveDrive(m_driverController));

        m_driverController.x().whileTrue(DumpFactory.runAll()); // aka "shoot"
        m_driverController.y().whileTrue(DumpFactory.runAtAdjustableRPM()); // aka "shoot" adust

        m_buttonController.povUp().onTrue(dump.incrementAdjustableTargetRPM());
        m_buttonController.povDown().onTrue(dump.decrementAdjustableTargetRPM());

        m_buttonController.leftBumper().whileTrue(IntakeFactory.runIntake());
        m_buttonController.leftTrigger().onTrue(intake.toggleIntake());
        m_buttonController.rightBumper().whileTrue(IntakeFactory.runOuttake());
    }

    public void setDefaultCommands() {
        intake.setDefaultCommand(intake.defaultCommand());
        swerve.setDefaultCommand(swerve.swerveDefaultCommand(m_driverController));
    }

    public Command getAutonomousCommand() {
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // Finally idle for the rest of auton
                drivetrain.applyRequest(() -> idle));
    }
}
