package frc.robot.subsystems.intake; 

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

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

public class IntakeSubsystem extends SubsystemBase {
    private final IntakeIO io; 
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final Angle pivotDeployedAngle = IntakeConstants.deployedAngle;
    private final Angle pivotStowedAngle = IntakeConstants.stowedAngle;

    private Angle setpointAngle = pivotStowedAngle;

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }   

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        Logger.recordOutput("Intake/PivotSetpoint", setpointAngle);

        Mechanism3d.measured.setIntake(inputs.PivotAngle);
        Mechanism3d.setpoints.setIntake(setpointAngle);

        if (Constants.getMode() == Mode.SIM) {
            MapleSimUtil.setIntakeRunning(isIntakeDeployed() && inputs.RollerAngularVelocity.gt(RadiansPerSecond.zero()));
        }
    }

    //Intake control methods
    private void setRollerVoltage(Voltage volts) {
        io.setRollerVoltage(volts);
    }

    private void setPivotAngle(Angle angle) {
        setpointAngle = angle;
        io.setPivotAngle(angle);
    }

    /**
     * Runs the roller.
     */
    private void startIntaking() {
        io.setRollerVoltage(IntakeConstants.rollerCollectVoltage);
    }

    /**
     * Stops the roller.
     */
    private void stopIntaking() {
        io.stopRoller();
    }

    /**
     * Stows the intake and hopper.
     */
    private void stow() {
        setPivotAngle(IntakeConstants.stowedAngle);
    }

    /**
     * Deploys the intake and hopper.
     */
    private void deploy() {
        setPivotAngle(IntakeConstants.deployedAngle);
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
    
    /**
     * Command that stows the intake and ends when it gets there.
     * @return
     */
    public Command stowCmd() {
        return Commands.runOnce(() -> stow())
            .andThen(Commands.waitUntil(() -> isIntakeStowed())
            .withTimeout(2)).withName("Stow Intake");
    }

    /**
     * Command that deploys the intake and ends when it gets there.
     * @return
     */
    public Command deployCmd() {
        return startDeployCmd()
            .andThen(Commands.waitUntil(this::isIntakeDeployed)
            .withTimeout(2)).withName("Deploy Intake");
    }

    /**
     * Command that starts the deployment of the intake, but doesnt wait until its done. 
     * @return
     */
    public Command startDeployCmd() {
        return runOnce(this::deploy);
    }

    /**
     * Command that runs the intake until the command ends.
     * @return The command
     */
    public Command runIntakeCmd() {
        return startEnd(this::startIntaking, this::stopIntaking);
    }

    public Command intakeSequence() {
        return runIntakeCmd().beforeStarting(
            deployCmd().unless(this::isIntakeDeployed)
        );
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
                (Voltage voltage)-> io.setPivotVoltage(voltage),
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
