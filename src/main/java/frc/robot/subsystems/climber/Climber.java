package frc.robot.subsystems.climber;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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


    //////////////////
    //Primary Methods/
    //////////////////
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


    /////////////////////
    //Secondary Methods//
    /////////////////////
    
    public void setDeployVoltage(Voltage voltage){
        io.setDeployVoltage(voltage);
    }

    public void setTwistVoltage(Voltage voltage){
        io.setTwistVoltage(voltage);
    }
    
    public void setDeployVelocity(AngularVelocity velocity){
        io.setDeployVelocity(velocity);
    }

    public void setTwistVelocity(AngularVelocity velocity){
        io.setTwistVelocity(velocity);
    }

    public void stopDeploy(){
        io.stopDeploy();
    }

    public void stopTwist(){
        io.stopTwist();
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

    public Command initializeTwist(){
        return Commands.runOnce(this::startTwist);
    }

    public Command startClimb(){
        return Commands.runOnce(this::climb);
    }

    public Command initializeClimber(){
        return initializeTwist().andThen(Commands.waitUntil(()->isTwistAtStart()))
        .beforeStarting(deployClimber().unless(this::isDeployed)).withName("Initialize and Deploy Climber");
    }

    public Command climbCommand() {
        return startClimb().andThen(Commands.waitUntil(() -> isTwistAtEnd()))
        .withTimeout(2).withName("Climb");
    }

    public Command stopTwistCommand() {
        return Commands.runOnce(this::stopTwist);
    }

    public Command stopDeployCommmand(){
        return Commands.runOnce(this::stopDeploy);
    }

    public Command resetClimber(){
        return Commands.sequence(
            initializeClimber(),
            stowClimber()
        );
    }

    /////////////////////////////////////////////////////
    ///Commands that aren't necessary, but can be used///
    /////////////////////////////////////////////////////
    
    public Command setDeployVoltageCommand(Voltage voltage) {
        return Commands.runOnce(() -> setDeployVoltage(voltage));
    }

    public Command setTwistVoltageCommand(Voltage voltage) {
        return Commands.runOnce(() -> setTwistVoltage(voltage));
    }

    public Command setDeployVelocityCommand(AngularVelocity velocity) {
        return Commands.runOnce(() -> setDeployVelocity(velocity));
    }

    public Command setTwistVelocityCommand(AngularVelocity velocity) {
        return Commands.runOnce(() -> setTwistVelocity(velocity));
    }

    //////////////////////////////////
    ///Sys ID Routine Configuration///
    //////////////////////////////////
    
    public final SysIdRoutine deploySysIdRoutine(){
        final Voltage stepVoltage_deploy= Volts.of(4); //This is temporary for the dynamic step voltage
        final Time timeOut_deploy= Seconds.of(10); //10 second timeout is considered default according Phoenix 6 documentation
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null, //Default ramp rate of the voltage of 1 V/s, according to Phoenix 6 documentation
                stepVoltage_deploy,
                timeOut_deploy,
                (state) -> Logger.recordOutput("state", state.toString()) //Logging the state of the routine
            ),
            new SysIdRoutine.Mechanism(
                (Voltage voltage)-> io.setDeployVoltage(voltage),
                null,
                this
            )
        );
    }

    public final SysIdRoutine twistSysIdRoutine(){
        final Voltage stepVoltage_twist= Volts.of(4); //This is temporary for the dynamic step voltage
        final Time timeOut_twist= Seconds.of(10); //10 second timeout is considered default according Phoenix 6 documentation
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null, //Default ramp rate of the voltage of 1 V/s, according to Phoenix 6 documentation
                stepVoltage_twist,
                timeOut_twist,
                (state) -> Logger.recordOutput("state", state.toString()) //Logging the state of the routine
            ),
            new SysIdRoutine.Mechanism(
                (Voltage voltage)-> io.setTwistVoltage(voltage),
                null,
                this
            )
        );
    }

    ////////////////////
    ///Sys ID Commands//
    ////////////////////
    
    public Command deploySysIdQuasistaticCommand(SysIdRoutine.Direction direction){
        return deploySysIdRoutine().quasistatic(direction);
    }
    
    public Command twistSysIdQuasistaticCommand(SysIdRoutine.Direction direction){
        return twistSysIdRoutine().quasistatic(direction);
    }
    
    public Command deploySysIdDynamicCommand(SysIdRoutine.Direction direction){
        return deploySysIdRoutine().dynamic(direction);
    }

    public Command twistSysIdDynamicCommand(SysIdRoutine.Direction direction){
        return twistSysIdRoutine().dynamic(direction);
    }    
}
