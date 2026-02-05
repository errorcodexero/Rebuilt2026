package frc.robot.subsystems.intake;

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
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.AngularAcceleration;


public final class IntakeConstants {
    public static final int rollerMotorCANID= 1;//Temporary
    public static final int pivotMotorCANID= 2; //Temporary

    public static final int currentLimit= 40; 
    public static final Time currentLimitTime= Seconds.of(1); //Temporary

    public static final double pivotKP= 0.5; //later for turning
    public static final double pivotKD= 0.04; //later for turning
    public static final double pivotKV= 0; //later for turning
    public static final double pivotKI= 0; //later for turning
    public static final double pivotKA= 0; //later for turning
    public static final double pivotKS= 0; //later for turning
    public static final double pivotKG= 0; //later for tuning
    
    public static final double rollerKP= 0.5; //later for roller speed
    public static final double rollerKD= 0; //change later
    public static final double rollerKV= 0; //change later
    public static final double rollerKI= 0; //change later
    public static final double rollerKA = 0; //change later
    public static final double rollerKS= 0; //change later
    public static final double rollerKG= 0; //change later

    public static final Angle stowedAngle= Degrees.of(0); //Temporary angle
    public static final Angle deployedAngle= Degrees.of(125); //Temporary angle
    public static final Angle pivotMinAngle= Degrees.of(-15); //Temporary angle
    public static final Angle pivotMaxAngle= Degrees.of(130); //Temporary angle
    public static final Angle pivotTargetAngle = Degrees.of(-25); //This variable can be changed if we want to run a command to tell intake to go to a specific position
    public static final Voltage pivotVoltage= Volts.of(2); //Temporary if we want to set the pivot voltage to a specifc value at all 

    public static final AngularVelocity pivotCruiseVelocity= DegreesPerSecond.of(90); //Temporary speed
    public static final AngularAcceleration pivotCruiseAcceleration= DegreesPerSecondPerSecond.of(180); //Temporary acceleration
    public static final double pivotMaxJerk= 0; //Temporary jerk

    public static final AngularVelocity rollerMaxVelocity= DegreesPerSecond.of(360); //Temporary speed to be changed as needed 
    public static final Voltage rollerCollectVoltage= Volts.of(6); //Temporary voltage

    public static final Angle pivotTolerance = Degrees.of(5); //Tolerance to compare current angle to target

    public static final double motorToPivotGearRatio = 2.0; //Unsure yet, will need it for the simulator
    public static final double rollerGearRatio = 1.0; //Unsure yet, will need it for the simulator
    public static final MomentOfInertia PIVOT_MOMENTOFINERTIA = KilogramSquareMeters.of(0.01);
    public static final MomentOfInertia ROLLER_MOMENTOFINERTIA = KilogramSquareMeters.of(0.01);
}