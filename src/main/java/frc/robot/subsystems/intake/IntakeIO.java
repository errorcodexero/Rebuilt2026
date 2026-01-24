package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public class IntakeIOInputs{
        public Angle PivotAngle= Degrees.of(0); //Temporary
        public AngularVelocity PivotAngularVelocity= DegreesPerSecond.of(0); //Value is Temporary
        public AngularVelocity RollerAngularVelocity= DegreesPerSecond.of(0); //Value is Temporary
        public Voltage RollerAppliedVolts= Volts.of(0); //Value is Temporary
        public Voltage PivotAppliedVolts= Volts.of(0); //Value is Temporary
        public Current RollerCurrentAmps= Amps.of(0); //Value is Temporary
        public Current PivotCurrentAmps= Amps.of(0); //Value is Temporary
    }
    
    public default void updateInputs(IntakeIOInputsAutoLogged inputs) {}
    public default void setRollerVoltage(Voltage volts) {}
    public default void setPivotAngle(Angle angle) {}

}