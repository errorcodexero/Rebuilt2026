package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class IntakeSysIdCommands {
    private IntakeSubsystem intake;
    private SysIdRoutine pivotSysIdRoutine= intake.pivotSysIdRoutine();
    private SysIdRoutine rollerSysIdRoutine= intake.rollerSysIdRoutine();
    
    public Command pivotSysIdQuasistaticCommand(SysIdRoutine.Direction direction){
        return pivotSysIdRoutine.quasistatic(direction);
    }

    public Command pivotSysIdDynamicCommand(SysIdRoutine.Direction direction){
        return pivotSysIdRoutine.dynamic(direction);
    }

    public Command rollerSysIdQuasistaticCommand(SysIdRoutine.Direction direction){
        return rollerSysIdRoutine.quasistatic(direction);
    }

    public Command rollerSysIdDynamicCommand(SysIdRoutine.Direction direction){
        return rollerSysIdRoutine.dynamic(direction);
    }
}
