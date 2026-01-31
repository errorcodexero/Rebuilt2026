package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.units.measure.MomentOfInertia;

public class ClimberConstants {
    public static final int motorOneId = 0;
    public static final int motorTwoId = 1;
    public static final int thriftyClimbId = 0;

    public static class Sim {
        public static final MomentOfInertia MOI = KilogramSquareMeters.zero();
        public static final double gearRatio = 1.0;
    }
}
