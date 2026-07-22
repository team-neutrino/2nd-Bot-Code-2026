package frc.robot.command_factories;

import static frc.robot.util.Constants.IntakeConstants.INTAKE_VOLTAGE;
import static frc.robot.util.Constants.IntakeConstants.OUTTAKE_VOLTAGE;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.util.Subsystems.intake;

public class IntakeFactory {

    // Figure out button bindings (for these and in general)

    public static Command runIntake() {
        return intake.runIntake(INTAKE_VOLTAGE);
    }

    public static Command runOuttake() {
        return intake.runIntake(OUTTAKE_VOLTAGE);
    }
}
