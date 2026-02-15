package frc.robot.subsystems.climber;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.MapleSimUtil;
import frc.robot.util.Mechanism3d;



public class Climber extends SubsystemBase {
    
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private final Angle deployDeployedAngle = ClimberConstants.deployDeployedAngle;
    private final Angle deployStowedAngle = ClimberConstants.deployStowedAngle;

     public Climber(ClimberIO io) {
        this.io=io;
    }

    @Override 
    public void periodic() {
        IO.updateInputs(inputs);
        Logger.processInputs(getName(), null);

        IO.applyOutputs(inputs);
        
    }

    public Command motorOneAngle(Angle angle){
        return Commands.runOnce(()-> {
            inputs.oneSetPoint= angle;
        });
    }
}
