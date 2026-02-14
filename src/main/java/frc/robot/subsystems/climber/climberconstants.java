package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import edu.wpi.first.units.measure.MomentOfInertia;

public class ClimberConstants {

    public static final int deployMotorID = 0; //For now
    public static final int climbMotorID = 1;  //For now

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

    public static final MomentOfInertia MOI=  KilogramSquareMeters.zero();
    public static final double gearatio =0.0; 
}
