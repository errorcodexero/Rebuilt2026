package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

public final class IntakeConstants {
    public static final int rollerMotorCANID= 1;//Temporary
    public static final int pivotMotorCANID= 2; //Temporary

    public static final int currentLimit= 40; //Temporary
    public static final Time currentLimitTime= Seconds.of(1); //Temporary


    public static final double pivotKP= 0.5; //later for turning
    public static final double pivotKD= 0.05; //later for turning
    public static final double pivotKV= 0; //later for turning
    public static final double pivotKI= 0; //later for turning

    public static final double rollerKP= 0.5; //later for roller speed
    public static final double rollerKD= 0; //change later
    public static final double rollerKV= 0; //change later
    public static final double rollerKI= 0; //change later

    public static final Angle stowedAngle= Degrees.of(-20); //Temporary
    public static final Angle deployedAngle= Degrees.of(-35); //Temporary
    public static final AngularVelocity pivotMaxVelocity= DegreesPerSecond.of(90); //Temporary
    public static final Voltage rollerVoltage= Volts.of(6); //Temporary
}