package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public final class IntakeConstants {
    public static final Time timeBeforeShake = Seconds.of(1.0);

    public static final int rollerMotorCANID = 9;
    public static final int pivotMotorCANID = 8;
    public static final int pivotEncoderCANID = 7;

    public static final Current rollerCurrentLimit = Amps.of(100);
    public static final Time rollerCurrentLimitTime = Milliseconds.of(200);

    public static final Current pivotCurrentLimit = Amps.of(40);
    public static final Time pivotCurrentLimitTime = Milliseconds.of(200);

    public static final double pivotKP = 100.0;
    public static final double pivotKD = 0;
    public static final double pivotKV = 7.0;
    public static final double pivotKI = 0;
    public static final double pivotKA = 0;
    public static final double pivotKS = 0;
    public static final double pivotKG = 0;
    
    public static final double rollerKP = 1.2;
    public static final double rollerKD = 0;
    public static final double rollerKV = 0.132;
    public static final double rollerKI = 0;
    public static final double rollerKA = 0;
    public static final double rollerKS = 0;
    public static final double rollerKG = 0;

    public static final Angle stowedAngle = Degrees.of(5);
    public static final Angle waitingAngle = Degrees.of(96);
    public static final Angle deployedAngle = Degrees.of(130);

    public static final int shakeDivisions = 2;

    public static final Time angleChangeDelay = Milliseconds.of(150);
    public static final Time angleDownDelay = Milliseconds.of(40);

    // Pivot Angle Constants
    public static final Angle pivotTolerance = Degrees.of(3); //Tolerance to compare current angle to target

    public static final boolean pivotEncoderInverted = false;
    public static final Angle pivotEncoderOffset = Degrees.of(-154.3) ;

    // Gear Ratios
    public static final double motorToEncoderGearRatio = (1.0/45.0) * (24.0/31.0);
    public static final double encoderToPivotGearRatio = 1.0;

    public static final double motorToPivotGearRatio = motorToEncoderGearRatio * encoderToPivotGearRatio;

    public static final Angle pivotMinAngle = Rotations.of(0) ;
    public static final Angle pivotMaxAngle = Degrees.of(135);

    public static final Voltage pivotHoldDownVoltage = Volts.of(4.0);

    public static final AngularVelocity pivotCruiseVelocity = RotationsPerSecond.of(0.5) ;
    public static final AngularAcceleration pivotCruiseAcceleration = RotationsPerSecondPerSecond.of(3);
    public static final double pivotMaxJerk = 0;

    public static final AngularVelocity rollerCollectVelocity = RotationsPerSecond.of(80);
    public static final AngularVelocity rollerShootVelocity = RotationsPerSecond.of(15);
    public static final Voltage rollerEjectVoltage = Volts.of(-6);

    public static final double rollerGearRatio = 1.0 ;
    public static final MomentOfInertia PIVOT_MOMENTOFINERTIA = KilogramSquareMeters.of(0.001);
    public static final MomentOfInertia ROLLER_MOMENTOFINERTIA = KilogramSquareMeters.of(0.001);
}