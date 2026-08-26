package frc.robot.util;

import frc.robot.subsystems.*;
import frc.robot.subsystems.network_tables.*;

public class Subsystems {
    public static final Intake intake = new IntakeNT();
    public static final Vision vision = new Vision();
    public static final Swerve swerve = new Swerve();
    public static final Dump dump = new DumpNT();
}
