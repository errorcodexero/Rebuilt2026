package frc.robot.subsystems.thriftyclimb;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

public class ThriftyClimbConstants {
    public static final int thriftyClimbId = 1;

    public static final Distance thriftyClimbSpoolDiameter = Inches.of(1.5 * 2);
    public static final double thriftyGearRatio = 1.0 / 50.0;
    public static final Distance thriftyClimbHeight = Inches.of(29.0);
    public static final Distance thriftyStowedHeight = Inches.of(20.0);

    /** This is used in simulation */
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.1);
}
