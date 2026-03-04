package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;

public final class IntakeConstants {
    public static final int rollerMotorCANID= 9;
    public static final int pivotMotorCANID= 8;

    public static final Current rollerCurrentLimit = Amps.of(180) ;
    public static final Current pivotCurrentLimit = Amps.of(80) ;
    public static final Time currentLimitTime= Seconds.of(1); //Temporary

    public static final double pivotKP= 4.0;
    public static final double pivotKD= 0;
    public static final double pivotKV= 0.222;
    public static final double pivotKI= 0;
    public static final double pivotKA= 0;
    public static final double pivotKS= 0;
    public static final double pivotKG= 0;
    
    public static final double rollerKP= 1.2;
    public static final double rollerKD= 0;
    public static final double rollerKV= 0.132;
    public static final double rollerKI= 0;
    public static final double rollerKA = 0;
    public static final double rollerKS= 0;
    public static final double rollerKG= 0;

    public static final Angle stowedAngle= Rotations.of(0);
    public static final Angle waitingAngle= Rotations.of(11.0);
    public static final Angle deployedAngle= Rotations.of(18.5);
    public static final Angle[] shootAngles= {Rotations.of(18.5), Rotations.of(11)};
    public static final Time angleChangeDelay = Seconds.of(0.2);

    public static final Angle pivotMinAngle= Rotations.of(0) ;
    public static final Angle pivotMaxAngle= Rotations.of(18.5);

    public static final AngularVelocity pivotCruiseVelocity= RotationsPerSecond.of(15);
    public static final AngularVelocity pivotCruiseAcceleration= RotationsPerSecond.of(100);
    public static final double pivotMaxJerk= 0;

    public static final AngularVelocity rollerCollectVelocity= RotationsPerSecond.of(32);
    public static final AngularVelocity rollerShootVelocity= RotationsPerSecond.of(15);
    public static final Voltage rollerEjectVoltage= Volts.of(-6);

    public static final Angle pivotTolerance = Rotations.of(1); //Tolerance to compare current angle to target

    public static final double motorToPivotGearRatio = 1.0 ;
    public static final double rollerGearRatio = 1.0 ;
    public static final MomentOfInertia PIVOT_MOMENTOFINERTIA = KilogramSquareMeters.of(0.001);
    public static final MomentOfInertia ROLLER_MOMENTOFINERTIA = KilogramSquareMeters.of(0.001);
}