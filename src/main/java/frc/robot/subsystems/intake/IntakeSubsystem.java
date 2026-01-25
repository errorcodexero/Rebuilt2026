package frc.robot.subsystems.intake; 

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.*;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io; 
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();


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


}