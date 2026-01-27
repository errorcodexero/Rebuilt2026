package frc.robot.subsystems.intake; 

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import com.ctre.phoenix6.SignalLogger;

import static edu.wpi.first.units.Units.*;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io; 
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final Angle pivotDeployedAngle = IntakeConstants.deployedAngle;
    private final Angle pivotStowedAngle = IntakeConstants.stowedAngle;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }   

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.recordOutput("Intake/PivotAngle", inputs.PivotAngle);
        Logger.recordOutput("Intake/RollerVoltage", inputs.RollerAppliedVolts);
        Logger.recordOutput("Intake/PivotAngularVelocity", inputs.PivotAngularVelocity);
        Logger.recordOutput("Intake/RollerAngularVelocity", inputs.RollerAngularVelocity);
        Logger.recordOutput("Intake/RollerCurrentAmps", inputs.RollerCurrentAmps);
        Logger.recordOutput("Intake/PivotCurrentAmps", inputs.PivotCurrentAmps);
        Logger.recordOutput("Intake/PivotAppliedVolts", inputs.PivotAppliedVolts);
    }

    //Intake control methods
    public void setRollerVoltage(Voltage volts) {
        io.setRollerVoltage(volts);
    }

    public void setPivotAngle(Angle angle) {
        io.setPivotAngle(angle);
    }

    public void stopRoller(){
        io.setRollerVoltage(Volts.of(0));
    }

    public void stowIntake(){
        setPivotAngle(IntakeConstants.stowedAngle);
    }

    public void deployIntake(){
        setPivotAngle(IntakeConstants.deployedAngle);
    }

    public void setRollerVelocity(AngularVelocity velocity) {
        io.setRollerVelocity(velocity);
    }   

    public void setPivotVoltage(Voltage voltage) {
        io.setPivotVoltage(voltage);
    }   

    public Angle getPivotAngle(){
        return inputs.PivotAngle;
    }

    public boolean isIntakeDeployed(){
        if(Degrees.of(getPivotAngle().in(Degrees)).isNear(pivotDeployedAngle, Degrees)){
            return true;
        } else {
            return false;
        }
    }

    public boolean isIntakeStowed(){
        if((getPivotAngle().in(Degrees)).isNear(pivotStowedAngle.in(Degrees))){
            return true;
        } else {
            return false;
        }
    }

    public boolean isPivotAtAngle(Angle angle){
        if(getPivotAngle().in(Degrees)==angle.in(Degrees)){
            return true;
        } else {
            return false;
        }
    }

    ////////////////////////////
    /// Sys ID Routine creation
    /// ////////////////////////

    public final SysIdRoutine pivotSysIdRoutine(){
        final Voltage stepVoltage_pivot= Volts.of(4); //This is temporary, for the dynamic step voltage
        final Time timeOut_pivot= Seconds.of(10); //Temporary timeout, considered default by Phoenix 6 documentation
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null, //Default ramp rate of the voltage of 1 V/s, according to Phoenix 6 documentation
                stepVoltage_pivot,
                timeOut_pivot,
                (state) -> SignalLogger.writeString("state", state.toString()) //Logging the state of the routine
            ),
            new SysIdRoutine.Mechanism(
                (Voltage voltage)-> setRollerVoltage(voltage),
                null, 
                this
            )
        );
    }

    public final SysIdRoutine rollerSysIdRoutine(){
        final Voltage stepVoltage_roller= Volts.of(4); //This is temporary, for the dynamic step voltage
        final Time timeOut_roller= Seconds.of(10); //Temporary timeout, considered default by Phoenix 6 documentation
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null, //Default ramp rate of the voltage of 1 V/s, according to Phoenix 6 documentation
                stepVoltage_roller,
                timeOut_roller,
                (state) -> SignalLogger.writeString("state", state.toString()) //Logging the state of the routine
            ),
            new SysIdRoutine.Mechanism(
                (Voltage Volts)-> setRollerVoltage(Volts),
                null, 
                this
            )
        );
    }
}
