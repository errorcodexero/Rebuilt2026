package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;
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
        public static final int climbMotorID = 1;  //For now
    }
    
    public class PID {
        public static final double deployKP= 0.0; //For now
        public static final double deployKI= 0.0; //For now
        public static final double deployKD= 0.0; //For now
        public static final double deployKS= 0.0; //For now
        public static final double deployKV= 0.0; //For now
        public static final double deployKG= 0.0; //For now
    
        public static final double climbKP= 0.0; //For now
        public static final double climbKI= 0.0; //For now
        public static final double climbKD= 0.0; //For now
        public static final double climbKS= 0.0; //For now
        public static final double climbKV= 0.0; //For now
        public static final double climbKG= 0.0; //For now
    }

    public class MotionMagic{
        public static final AngularVelocity depolyCruiseVelocity=DegreesPerSecond.of(90);
        public static final AngularAcceleration deployAngularAcceleration= DegreesPerSecondPerSecond.of(180);
        public static final double deployJerk = 0;

        public static final AngularVelocity climbCruiseVelocity=DegreesPerSecond.of(90);
        public static final AngularAcceleration climbAngularAcceleration= DegreesPerSecondPerSecond.of(180);
        public static final double climbJerk = 0;
    }

        public class AngleSetpoints {
        public static final Angle deployMinAngle= Degrees.of(0.0); //For now
        public static final Angle deployMaxAngle= Degrees.of(0.0); //For now
        public static final Angle stowedAngle= Degrees.of(0.0); //For now
        public static final Angle deployedAngle= Degrees.of(0.0); //For now

        public static final Angle climbMinAngle= Degrees.of(0.0); //For now
        public static final Angle climbMaxAngle= Degrees.of(0.0); //For now
        public static final Angle climbStartAngle= Degrees.of(0.0); //For now
        public static final Angle climbEndAngle= Degrees.of(0.0); //For now
    }

    public class VoltageSetpoints{
        public static final Voltage deployMaxVoltage= Volts.of(0.0); //For now
        public static final Voltage climbMaxVoltage= Volts.of(0.0); //For now
    }
    

    public class VelocitySetpoints{
        public static final AngularVelocity deployMaxAngularVelocity= DegreesPerSecond.of(0.0); //For now
        public static final AngularVelocity climbMaxAngularVelocity= DegreesPerSecond.of(0.0); //For now
    }

    public class MOI{
        public static final MomentOfInertia deployMOI=  KilogramSquareMeters.of(0.0); //For now
        public static final MomentOfInertia climbMOI= KilogramSquareMeters.of(0.0); //For now
    }

    public class GearRatios{
        public static final double deployGearRatio =0.0;
        public static final double climbGearRatio= 0.0;
    }
}
