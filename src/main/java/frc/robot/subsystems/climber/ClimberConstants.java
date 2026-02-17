package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.AngularAcceleration;

public class ClimberConstants {
    public class CANID {
        public static final int deployMotorID = 0; //For now
        public static final int twistMotorID = 1;  //For now
    }
    
    public class PID {
        public static final double deployKP= 0.5; //For now
        public static final double deployKI= 0.04; //For now
        public static final double deployKD= 0.0; //For now
        public static final double deployKS= 0.0; //For now
        public static final double deployKV= 0.0; //For now
        public static final double deployKG= 0.0; //For now
    
        public static final double twistKP= 0.0; //For now
        public static final double twistKI= 0.0; //For now
        public static final double twistKD= 0.0; //For now
        public static final double twistKS= 0.0; //For now
        public static final double twistKV= 0.0; //For now
        public static final double twistKG= 0.0; //For now
    }

    public class MotionMagic{
        public static final AngularVelocity deployCruiseVelocity= DegreesPerSecond.of(90);
        public static final AngularAcceleration deployAngularAcceleration= DegreesPerSecondPerSecond.of(180);
        public static final double deployJerk = 0;

        public static final AngularVelocity twistCruiseVelocity=DegreesPerSecond.of(90);
        public static final AngularAcceleration twistAngularAcceleration= DegreesPerSecondPerSecond.of(180);
        public static final double twistJerk = 0;
    }

        public class AngleSetpoints {
        public static final Angle deployMinAngle= Degrees.of(0.0); //For now
        public static final Angle deployMaxAngle= Degrees.of(100); //For now
        public static final Angle stowedAngle= Degrees.of(45); //For now
        public static final Angle deployedAngle= Degrees.of(90); //For now

        public static final Angle twistMinAngle= Degrees.of(0.0); //For now
        public static final Angle twistMaxAngle= Degrees.of(360); //For now
        public static final Angle twistStartAngle= Degrees.of(20); //For now
        public static final Angle twistL3Angle= Degrees.of(340); //For now
        public static final Angle twistL1Angle= Degrees.of(90); //For now
    }

    public class CurrentLimits {
        public static final int currentLimit= 40; 
    }

    public class VoltageSetpoints{
        public static final Voltage deployMaxVoltage= Volts.of(0.0); //For now
        public static final Voltage twistMaxVoltage= Volts.of(0.0); //For now
    }
    

    public class VelocitySetpoints{
        public static final AngularVelocity deployMaxAngularVelocity= DegreesPerSecond.of(0.0); //For now
        public static final AngularVelocity twistMaxAngularVelocity= DegreesPerSecond.of(0.0); //For now
    }

    public class MOI{
        public static final MomentOfInertia deployMOI=  KilogramSquareMeters.of(0.01); //For now
        public static final MomentOfInertia twistMOI= KilogramSquareMeters.of(0.01); //For now
    }

    public class GearRatios{
        public static final double deployGearRatio =1; //For now
        public static final double twistGearRatio= 1; //For now
    }

    public class Tolerances{
        public static final Angle deployTolerance= Degrees.of(5);
        public static final Angle twistTolerance= Degrees.of(5);
    }
}
