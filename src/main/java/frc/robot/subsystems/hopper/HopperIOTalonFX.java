package frc.robot.subsystems.hopper;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;


import com.ctre.phoenix6.hardware.*;

import static edu.wpi.first.units.Units.Amps;

public class HopperIOTalonFX implements HopperIO {
    
    //Creating Motor Objects
    protected final TalonFX feederMotor;
    protected final TalonFX scramblerMotor;

    //Feeder Control Requests
    private final VoltageOut feederVoltageRequest = new VoltageOut(0);
    private final VelocityVoltage feederVelocityRequest  = new VelocityVoltage(0.0);

    //Scrambler Control Requests
    private final VoltageOut scramblerVoltageRequest = new VoltageOut(0);
    private final VelocityVoltage scramblerVelocityRequest  = new VelocityVoltage(0.0);

    //Feeder Status Signals
    private StatusSignal<AngularVelocity> feederAngularVelocitySignal;
    private StatusSignal<Voltage> feederVoltageSignal;
    private StatusSignal<Current> feederCurrentSignal; 

    //Scrambler Status Signals
    private StatusSignal<AngularVelocity> scramblerAngularVelocitySignal;   
    private StatusSignal<Voltage> scramblerVoltageSignal;
    private StatusSignal<Current> scramblerCurrentSignal;


    public HopperIOTalonFX() {
        
        //Feeder Motor Configuration
        feederMotor = new TalonFX(HopperConstants.feederMotorCANID);
        TalonFXConfiguration feederConfig = new TalonFXConfiguration();

        feederConfig.Slot0.kP = HopperConstants.feederKP;
        feederConfig.Slot0.kI = HopperConstants.feederKI;
        feederConfig.Slot0.kD = HopperConstants.feederKD;
        feederConfig.Slot0.kS = HopperConstants.feederKS;
        feederConfig.Slot0.kV = HopperConstants.feederKV;
        feederConfig.Slot0.kA = HopperConstants.feederKA;

        tryUntilOk(5, () -> feederMotor.getConfigurator().apply(feederConfig, 0.25));

        feederConfig.CurrentLimits.StatorCurrentLimit = HopperConstants.feederCurrentLimit.in(Amps);

        //Scrambler Motor Configuration
        scramblerMotor = new TalonFX(HopperConstants.scramblerMotorCANID);
        TalonFXConfiguration scramblerConfig = new TalonFXConfiguration();

        scramblerConfig.Slot0.kP = HopperConstants.scramblerKP;
        scramblerConfig.Slot0.kI = HopperConstants.scramblerKI;
        scramblerConfig.Slot0.kD = HopperConstants.scramblerKD;
        scramblerConfig.Slot0.kS = HopperConstants.scramblerKS;
        scramblerConfig.Slot0.kV = HopperConstants.scramblerKV;
        scramblerConfig.Slot0.kA = HopperConstants.scramblerKA;

        tryUntilOk(5, () -> scramblerMotor.getConfigurator().apply(scramblerConfig, 0.25));

        scramblerConfig.CurrentLimits.StatorCurrentLimit = HopperConstants.scramblerCurrentLimit.in(Amps);
        
        //Status Signals Initialization
        feederAngularVelocitySignal = feederMotor.getVelocity();
        feederVoltageSignal = feederMotor.getSupplyVoltage();
        feederCurrentSignal = feederMotor.getStatorCurrent();

        scramblerAngularVelocitySignal = scramblerMotor.getVelocity();
        scramblerVoltageSignal = scramblerMotor.getSupplyVoltage();
        scramblerCurrentSignal = scramblerMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(0.0, 
            feederAngularVelocitySignal, feederVoltageSignal, feederCurrentSignal, 
            scramblerAngularVelocitySignal, scramblerVoltageSignal, scramblerCurrentSignal);
        
        //Optimize Bus Utilization
        feederMotor.optimizeBusUtilization();
        scramblerMotor.optimizeBusUtilization();

    }
    @Override
    public void updateInputs(HopperIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            feederAngularVelocitySignal, feederVoltageSignal, feederCurrentSignal, 
            scramblerAngularVelocitySignal, scramblerVoltageSignal, scramblerCurrentSignal);
        
        inputs.feederVelocity = feederAngularVelocitySignal.getValue();
        inputs.feederVoltage = feederVoltageSignal.getValue();
        inputs.feederCurrent = feederCurrentSignal.getValue();
        inputs.scramblerVelocity = scramblerAngularVelocitySignal.getValue();
        inputs.scramblerVoltage = scramblerVoltageSignal.getValue();
        inputs.scramblerCurrent = scramblerCurrentSignal.getValue();
    }

    @Override
    public void setFeederVoltage(Voltage voltage) {
        feederMotor.setControl(feederVoltageRequest.withOutput(voltage));
    }

    @Override
    public void setFeederVelocity(AngularVelocity velocity) {
        feederMotor.setControl(feederVelocityRequest.withVelocity(velocity));
    }

    @Override
    public void setScramblerVoltage(Voltage voltage) {
        scramblerMotor.setControl(scramblerVoltageRequest.withOutput(voltage));
    }

    @Override
    public void setScramblerVelocity(AngularVelocity velocity) {
        scramblerMotor.setControl(scramblerVelocityRequest.withVelocity(velocity));
    }

    public void stopFeeder() {
        feederMotor.setControl(feederVoltageRequest.withOutput(0));
    }

    public void stopScrambler() {
        scramblerMotor.setControl(scramblerVoltageRequest.withOutput(0));
    }
}