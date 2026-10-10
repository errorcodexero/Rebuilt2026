package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public class IntakeIOInputs{
        public boolean pivotConnected = false;
        public Angle pivotAngle = Degrees.of(0);
        public Angle pivotCancoderPosition = Degrees.of(0);
        public AngularVelocity pivotAngularVelocity = DegreesPerSecond.of(0); 
        public AngularVelocity pivotCancoderVelocity = DegreesPerSecond.of(0);
        public Voltage pivotAppliedVolts = Volts.of(0);
        public Current pivotCurrentAmps = Amps.of(0); 
        public Temperature pivotTemp = Celsius.zero();

        public boolean rollerConnected = false;
        public AngularVelocity rollerAngularVelocity = DegreesPerSecond.of(0); 
        public Voltage rollerAppliedVolts = Volts.of(0); 
        public Current rollerCurrentAmps = Amps.of(0); 
        public Temperature rollerTemp = Celsius.zero();
    }
    
    public default void updateInputs(IntakeIOInputsAutoLogged inputs) {}

    public default void setRollerVoltage(Voltage volts) {}

    public default void setRollerVelocity(AngularVelocity velocity) {}

    public default void setPivotVoltage(Voltage voltage) {} /*Switched to voltage since velocity control will likely not be needed, 
                                                                    and voltage will be needed for SysId routines*/
    public default void setPivotAngle(Angle angle) {}

    public default void stopRoller() {}

}