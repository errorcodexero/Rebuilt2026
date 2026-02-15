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
    private final Angle deployDeployedAngle = ClimberConstants.AngleSetpoints.deployedAngle;
    private final Angle deployStowedAngle = ClimberConstants.AngleSetpoints.stowedAngle;
    private final Angle twistStartAngle= ClimberConstants.AngleSetpoints.twistStartAngle;
    private final Angle twistEndAngle= ClimberConstants.AngleSetpoints.twistEndAngle;

     public Climber(ClimberIO io) {
        this.io=io;
    }

    @Override 
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        Logger.recordOutput("Climber/DeployMotor_DeployedAngleSetpoint", deployDeployedAngle);
        Logger.recordOutput("Climber/DeployMotor_StowedAngleSetpoint", deployStowedAngle);
        Logger.recordOutput("Climber/TwistMotor_StartAngleSetpoint", twistStartAngle);
        Logger.recordOutput("Climber/TwistMotor_EndAngleSetpoint", twistEndAngle);
    }

    public void setDeployMotorAngle(Angle angle) {
        io.setDeployAngle(angle);
    }

    public void setTwistMotorAngle(Angle angle) {
        io.setTwistAngle(angle);
    }

    public void deploy(){
        setDeployMotorAngle(deployDeployedAngle);
    }

    public void stow() {
        setDeployMotorAngle(deployStowedAngle);
    }

    public void setTwistAngle(Angle angle) {
        io.setTwistAngle(angle);
    }

    public void startTwist(){
        setTwistMotorAngle(twistStartAngle);
    }

    public void climb(){
        setTwistMotorAngle(twistEndAngle);
    }

    public boolean isDeployAtAngle(Angle angle) {
        return inputs.deployPosition.isNear(angle, ClimberConstants.Tolerances.deployTolerance);
    }

    public boolean isTwistAtAngle(Angle angle) {
        return inputs.twistPosition.isNear(angle, ClimberConstants.Tolerances.twistTolerance);
    }

    public boolean isDeployed() {
        return isDeployAtAngle(deployDeployedAngle);
    }

    public boolean isStowed() {
        return isDeployAtAngle(deployStowedAngle);
    }

    public boolean isTwistAtStart() {
        return isTwistAtAngle(twistStartAngle);
    }

    public boolean isTwistAtEnd() {
        return isTwistAtAngle(twistEndAngle);
    }

    /////////////
    ///Commands//
    /////////////
    
    public Command startDeploy(){
        return Commands.runOnce(this::deploy);
    }

    public Command startStow(){
        return Commands.runOnce(this::stow);
    }

    public Command deployClimber(){
        return startDeploy().andThen(Commands.waitUntil(()->isDeployed()))
        .withTimeout(2).withName("Deploy Climber");
    }

    public Command stowClimber(){
        return startStow().andThen(Commands.waitUntil(()->isStowed()))
        .withTimeout(2).withName("Stow Climber");
    }

    
}
