package frc.robot.subsystems.intake; 

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.util.MapleSimUtil;
import frc.robot.util.Mechanism3d;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

import static edu.wpi.first.units.Units.*;

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io; 
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final Angle pivotDeployedAngle = IntakeConstants.deployedAngle;
    private final Angle pivotStowedAngle = IntakeConstants.stowedAngle;

    private Angle setpointAngle = pivotStowedAngle;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
        Logger.processInputs("Intake", inputs);

        if (Constants.getMode() == Mode.SIM) {
            MapleSimUtil.createIntake();
        }
    }   

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        Logger.recordOutput("Intake/Setpoint", setpointAngle);

        Mechanism3d.measured.setIntake(inputs.PivotAngle);
        Mechanism3d.setpoints.setIntake(setpointAngle);

        if (Constants.getMode() == Mode.SIM) {
            MapleSimUtil.setIntakeRunning(isIntakeDeployed());
        }
    }

    //Intake control methods
    public void setRollerVoltage(Voltage volts) {
        io.setRollerVoltage(volts);
    }

    public void setPivotAngle(Angle angle) {
        setpointAngle = angle;
        io.setPivotAngle(angle);
    }

    public void stopRoller(){
        io.stopRoller();
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

    public boolean isIntakeDeployed() {
        return isPivotAtAngle(pivotDeployedAngle);
    }

    public boolean isIntakeStowed(){
        return isPivotAtAngle(pivotStowedAngle);
    }

    public boolean isPivotAtAngle(Angle angle){
        return inputs.PivotAngle.isNear(angle, IntakeConstants.pivotTolerance);
    }

    /////////////
    ///Commands//
    /////////////
    
    //////////////////////////////////////////////////////
    /// First stow the intake, and then wait until it is stowed
    //////////////////////////////////////////////////////
    public Command stowIntakeCommand() {
        return Commands.runOnce(() -> stowIntake())
        .andThen(Commands.waitUntil(() -> isIntakeStowed())
        .withTimeout(2)).withName("Stow Intake");
    }

    //////////////////////////////////////////////////////
    /// First deploy the intake, and then wait until it is deployed
    //////////////////////////////////////////////////////
    public Command deployIntakeCommand() {
        return Commands.runOnce(() -> deployIntake())
        .andThen(Commands.waitUntil(() -> isIntakeDeployed())
        .withTimeout(2)).withName("Deploy Intake");
    }

    /////////////////////////
    /// Stop the rollers once
    /////////////////////////
    public Command stopRollerCommand() {
        return Commands.runOnce(() -> stopRoller())
        .withName("Stop Roller");
    }

    ////////////////////////////////////////////////////////
    /// Continously set the roller voltage until interrupted
    ////////////////////////////////////////////////////////
    public Command setRollerVoltageCommand(Voltage volts) {
        return Commands.run(() -> setRollerVoltage(volts))
        .withName("Set Roller Voltage");
    }

    //////////////////////////////////////////////////////////////////
    /// Continously move the rollers at set velocity until interrupted
    //////////////////////////////////////////////////////////////////
    public Command setRollerVelocityCommand(AngularVelocity velocity) {
        return Commands.run(() -> setRollerVelocity(velocity))
        .withName("Set Roller Velocity");
    }

    ////////////////////////////////////////////////////////////////
    /// Continously move the pivot at set velocity until interrupted
    ////////////////////////////////////////////////////////////////
    public Command setPivotVoltageCommand(Voltage voltage) {
        return Commands.run(() -> setPivotVoltage(voltage))
        .withName("Set Pivot Voltage");
    }

    /////////////////////////////////////////////////////////////
    /// First set the angle target, then wait until it is reached
    /////////////////////////////////////////////////////////////
    public Command setPivotAngleCommand(Angle angle) {
        return Commands.runOnce(() ->setPivotAngle(angle))
        .andThen(Commands.waitUntil(() -> isPivotAtAngle(angle))
        .withTimeout(2)).withName("Set Pivot Angle");
    }

    ////////////////////////////////
    ///Deploy and intake in parallel
    ////////////////////////////////
    public Command intakeDeployCommand(){
        return Commands.runOnce(() -> setRollerVoltage(IntakeConstants.rollerCollectVoltage))
        .andThen(Commands.runOnce(()->deployIntake() ));
    }

    ////////////////////////////////////////
    ///Stow and stop the rollers in parallel
    ////////////////////////////////////////
    public Command stopStowCommand(){
        return Commands.runOnce(() -> stopRollerCommand())
        .andThen(Commands.runOnce(()-> stowIntakeCommand()));
    }


    ////////////////////////////
    /// Sys ID Routine creation/
    /// ////////////////////////

    public final SysIdRoutine pivotSysIdRoutine(){
        final Voltage stepVoltage_pivot= Volts.of(4); //This is temporary, for the dynamic step voltage
        final Time timeOut_pivot= Seconds.of(10); //Temporary timeout, considered default by Phoenix 6 documentation
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null, //Default ramp rate of the voltage of 1 V/s, according to Phoenix 6 documentation
                stepVoltage_pivot,
                timeOut_pivot,
                (state) -> Logger.recordOutput("state", state.toString()) //Logging the state of the routine
            ),
            new SysIdRoutine.Mechanism(
                (Voltage voltage)-> setPivotVoltage(voltage),
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
                (state) -> Logger.recordOutput("state", state.toString()) //Logging the state of the routine
            ),
            new SysIdRoutine.Mechanism(
                (Voltage Volts)-> setRollerVoltage(Volts),
                null, 
                this
            )
        );
    }

    
    ///////////////////
    // Sys Id Commands/
    ///////////////////

    public Command pivotSysIdQuasistaticCommand(SysIdRoutine.Direction direction){
        return pivotSysIdRoutine().quasistatic(direction);
    }

    public Command pivotSysIdDynamicCommand(SysIdRoutine.Direction direction){
        return pivotSysIdRoutine().dynamic(direction);
    }

    public Command rollerSysIdQuasistaticCommand(SysIdRoutine.Direction direction){
        return rollerSysIdRoutine().quasistatic(direction);
    }

    public Command rollerSysIdDynamicCommand(SysIdRoutine.Direction direction){
        return rollerSysIdRoutine().dynamic(direction);
    }
}
