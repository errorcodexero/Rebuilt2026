package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;

public final class IntakeConstants {
    public static final int rollerMotorID= 1;//Temporary
    public static final int pivotMotorID= 2; //Temporary

    public static final double pivotKP= 5.0; 
    public static final double pivotKD= 0; //later for turning
    public static final double pivotKV= 0; //later for turning
    public static final double pivotKI= 0; //later for turning

    public static final Angle stowedAngle= Rotations.of(0); //Temporary
    public static final Angle deployedAngle= Rotations.of(0.1); //Temporary
    public static final Voltage rollerVoltage= Volts.of(6);
}