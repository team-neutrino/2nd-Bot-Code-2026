// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.List;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public static class RioConstants {
        public static final CANBus RIO_BUS = new CANBus("rio");
    }

    public static class ShooterConstants {

    }

    public static class IndexConstants {

    }

    public static class IntakeConstants {

    }

    public static class LimelightConstants {

    }

    public static class SwerveConstants {

    }

    public static class FieldMeasurementConstants {
        public static final double ALLIANCE_ZONE_BLUE = 3.978;
        public static final double ALLIANCE_ZONE_RED = 12.563;
        public static final double MID_FIELD_Y = 4.034663;
        public static final double MID_FIELD_X = 8.270494;
        public static final Pose2d RED_HUB = new Pose2d(11.915394, 4.034663, new Rotation2d(0));
        public static final Pose2d BLUE_HUB = new Pose2d(4.625594, 4.034663, new Rotation2d(0));
        public static final Pose2d SHUTTLE_TARGET_TOP_RED = new Pose2d(16.5, 6, new Rotation2d(0));
        public static final Pose2d SHUTTLE_TARGET_BOTTOM_RED = new Pose2d(16.5, 2, new Rotation2d(0));
        public static final Pose2d SHUTTLE_TARGET_TOP_BLUE = new Pose2d(0, 7, new Rotation2d(0));
        public static final Pose2d SHUTTLE_TARGET_BOTTOM_BLUE = new Pose2d(0, 1, new Rotation2d(0));
        public static final Pose2d ALLIANCE_WALL_TARGET_RED = new Pose2d(16.5, 4.034663, new Rotation2d(0));
        public static final Pose2d ALLIANCE_WALL_TARGET_BLUE = new Pose2d(0.0, 4.034663, new Rotation2d(0));
        public static final double ZERO = 0;
        public static final double FIELD_DIMENSION_X = 16.540988;
        public static final double FIELD_DIMENSION_Y = 8.069326;
        public static final double BLUE_DEPOT_BUMP_NEUTRAL_X = 5.222494;
        public static final double BLUE_DEPOT_BUMP_ALLIANCE_X = 4.061714;
        public static final double RED_DEPOT_BUMP_NEUTRAL_X = 11.318494;
        public static final double RED_DEPOT_BUMP_ALLIANCE_X = 12.479274;
        public static final double BLUE_BUMP_CENTER_X = 4.625594;
        public static final double RED_BUMP_CENTER_X = 11.915394;
        public static final double DEPOT_BUMP_Y = 6.477508;
        public static final double OUTPOST_BUMP_Y = 1.53289;
        public static final double NET_TOP = 4.763135;
        public static final double NET_BOTTOM = 3.279521;
    }

    public static class AprilTagConstants {
        public static final List<Integer> ALL_HUB_TAGS = List.of(
                2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27);
    }
}
