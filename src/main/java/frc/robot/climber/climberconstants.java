package frc.robot.climber;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import edu.wpi.first.units.measure.MomentOfInertia;

public class ClimberConstants {

public static final int MotoroneID = 0;
public static final int MotortwoID = 1; 

    public static class sim {
        public static final MomentOfInertia
        MOI=  KilogramSquareMeters.zero();
        public static final double gearatio =1.0; 
    }
}
