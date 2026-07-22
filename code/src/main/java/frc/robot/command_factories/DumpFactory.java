package frc.robot.command_factories;

import static frc.robot.util.Constants.DumpConstants.*;
import static frc.robot.util.Subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;

public class DumpFactory {

    public static Command runRollers() {
        return dump.setRollerRPM(DEFAULT_ROLLER_RPM);
    }

    public static Command runFloor() {
        return dump.setFloorRPM(DEFAULT_FLOOR_RPM);
    }

    public static Command runBoth() {
        return dump.setRollerRPM(DEFAULT_ROLLER_RPM).alongWith(dump.setFloorRPM(DEFAULT_FLOOR_RPM));
    }
}
