package frc.robot.climber;
import java.util.InputMismatchException;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class climber extends SubsystemBase {
    
    private final climberIO io;
    private final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged(); 
    private final climberconstants outputs = new ClimberOutputs();
    
    public climber(climberIO io){
        this.io=io;
    }
    @Override 
    public void periodic() {
        io.updateInput(inputs);
        Logger.processInputs(getName(), null);
        
        ///periodic logic function for climber 
        io.applytheOutputs(Climber outputs);
    }
    public Command motorOneAngle(Angle angle){
        return Commands.runOnce(()->; {outputs.oneSetPoint= angle;});
    }
}


