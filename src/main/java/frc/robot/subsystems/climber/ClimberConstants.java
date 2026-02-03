package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

public class ClimberConstants {
    public static final int motorOneId = 0;
    public static final int motorTwoId = 1;
    public static final int thriftyClimbId = 1;

    public static final Distance thriftyClimbSpoolDiameter = Inches.of(1.5 * 2);
    public static final double thriftyGearRatio = 50.0;
    public static final Distance thriftyClimbHeight = Inches.of(29.0);
    public static final Distance thriftyStowedHeight = Inches.of(20.0);

    public static class Sim {
        public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.001);
        public static final double gearRatio = 1.0;
    }
}
