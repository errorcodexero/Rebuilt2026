// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import java.util.Set;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;

/**
* This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
* on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
* (log replay from a file).
*/
public final class Constants {

    /**
     * CONFIGURATION
     */
    
    // Sets the currently running robot. Change to SIMBOT when running the
    // desktop physics simulation so AdvantageKit runs in SIM mode instead of
    // falling back to REPLAY.
    //REMEMBER TO CHANGE BACK TO COMPETITION!
    private static final RobotType robotType = RobotType.COMPETITION;

    public static final boolean spawnLessFuelInSim = true;

    public static final boolean shootOnMove = true;

    public static final double speedMultiplierShooting = 0.4; 
    public static final double speedMultiplierFerrying = 0.8; 

    public static class DriveConstants {
        public static final double slowModeJoystickMultiplier = 0.4;
    }
    
    public static class FieldConstants {
        public static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        public static final Set<Pose2d> trenches = Set.of(
            getTagPose(7),
            getTagPose(12),
            getTagPose(23),
            getTagPose(28)
        );

        private static Pose2d getTagPose(int id) {
            return layout.getTagPose(id).orElseThrow().toPose2d();
        }
    }

    /**
     * ROBOT STATE
     */
    
    public static enum Mode {
        /** Running on a real robot. */
        REAL,
        
        /** Running a physics simulator. */
        SIM,
        
        /** Replaying from a log file. */
        REPLAY
    }

    public static enum RobotType {
        COMPETITION, // The competition robot (with aluminum base)

        /** The Sim Bot */
        SIMBOT
    }

    // This is only a fallback! This will not change the robot type.
    private static final RobotType defaultRobotType = RobotType.COMPETITION;

    private static final Alert invalidRobotType = new Alert(
        "Invalid RobotType selected. Defaulting to " + defaultRobotType.toString(),
        AlertType.kWarning
    );

    public static RobotType getRobot() {
        if (RobotBase.isReal() && robotType == RobotType.SIMBOT) {
            invalidRobotType.set(true);
            return defaultRobotType;
        }

        return robotType;
    }

    public static final Mode getMode() {
        return switch(getRobot()) {
            case SIMBOT -> Mode.SIM;
            default -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
        };
    }
}
