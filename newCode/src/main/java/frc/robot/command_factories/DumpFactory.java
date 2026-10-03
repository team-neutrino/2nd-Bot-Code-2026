package frc.robot.command_factories;

import static frc.robot.util.Constants.DumpConstants.*;
import static frc.robot.util.Subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;

public class DumpFactory {

    private static double adjustRPM = 4000;

    public static Command runKicker() {
        return dump.setKickerRPM(DEFAULT_KICKER_RPM);
    }

    public static Command runRollers() {
        return dump.setRollerRPM(DEFAULT_ROLLER_RPM);
    }

    public static Command runFloor() {
        return dump.runFloor(DEFAULT_FLOOR_VOLTAGE);
    }

    public static Command runFloorReverse() {
        return dump.runFloor(-DEFAULT_FLOOR_VOLTAGE);
    }

    public static Command runAll() {
        return dump.runAll(DEFAULT_KICKER_RPM, DEFAULT_ROLLER_RPM, DEFAULT_FLOOR_VOLTAGE);
    }

    public static Command runAllSlower() {
        return dump.runAll(DEFAULT_KICKER_RPM / 2, DEFAULT_ROLLER_RPM / 2, DEFAULT_FLOOR_VOLTAGE);
    }

    public static Command runAtAdjustableRPM() {
        return dump.runAdjust(DEFAULT_KICKER_RPM, dump.getAdjustableTargetRPM(), DEFAULT_FLOOR_VOLTAGE);
    }

    public static Command incrementAdjustableRPM() {
        return dump.incrementAdjustableTargetRPM();
    }

    public static Command decrementAdjustableRPM() {
        return dump.decrementAdjustableTargetRPM();
    }
}