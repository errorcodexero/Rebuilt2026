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
        public Angle PivotAngle= Degrees.of(0);
        public AngularVelocity PivotAngularVelocity= DegreesPerSecond.of(0); 

        public AngularVelocity RollerAngularVelocity= DegreesPerSecond.of(0); 
        public Voltage RollerAppliedVolts= Volts.of(0); 

        public Voltage PivotAppliedVolts= Volts.of(0);
        public Current RollerCurrentAmps= Amps.of(0); 
        public Current PivotCurrentAmps= Amps.of(0); 
    }
    
    public default void updateInputs(IntakeIOInputsAutoLogged inputs) {}

    public default void setRollerVoltage(Voltage volts) {}

    public default void setRollerVelocity(AngularVelocity velocity) {}

    public default void setPivotVoltage(Voltage voltage) {} /*Switched to voltage since velocity control will likely not be needed, 
                                                                    and voltage will be needed for SysId routines*/

    public default void setPivotAngle(Angle angle) {}

    public default void stopRoller() {}

}