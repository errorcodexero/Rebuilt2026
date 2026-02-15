  package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.AngularVelocity;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal; 
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.CANBus; 
import com.ctre.phoenix6.configs.TalonFXConfiguration;

//import frc.robot.climber.climberIO.ClimberInputs;


public class ClimberIOTalonFX implements ClimberIO {
    //Motor object creation
    private final TalonFX deployMotor;
    private final TalonFX twistMotor; 

    //Deploy motor control requests
    private final MotionMagicVoltage deployAngleRequest= new MotionMagicVoltage(Degrees.of(0));
    private final VoltageOut deployVoltageRequest= new VoltageOut(Volts.of(0));
    private final VelocityVoltage deployVelocityRequest= new VelocityVoltage(DegreesPerSecond.of(0));

    //Twist motor control requests
    private final MotionMagicVoltage twistAngleRequest= new MotionMagicVoltage(Degrees.of(0));
    private final VoltageOut twistVoltageRequest= new VoltageOut(Volts.of(0));
    private final VelocityVoltage twistVelocityRequest= new VelocityVoltage(DegreesPerSecond.of(0));
        
    //Deploy motor status signals
    private final StatusSignal<Angle> deployPositionSig;
    private final StatusSignal<Voltage> deployVoltageSig;
    private final StatusSignal<Current> deployCurrentSig;
    private final StatusSignal<AngularVelocity> deployVelocitySig;

    //Twist motor status signals
    private final StatusSignal<Angle> twistPositionSig;
    private final StatusSignal<Voltage> twistVoltageSig;
    private final StatusSignal<Current> twistCurrentSig;
    private final StatusSignal<AngularVelocity> twistVelocitySig;
    
    public ClimberIOTalonFX(CANBus deployBus, CANBus twistBus){
        deployMotor= new TalonFX(ClimberConstants.CANID.deployMotorID, deployBus);
        twistMotor = new TalonFX(ClimberConstants.CANID.twistMotorID, twistBus);

        TalonFXConfiguration deployConfig= new TalonFXConfiguration();

        deployConfig.MotorOutput.NeutralMode= NeutralModeValue.Brake;

        deployConfig.CurrentLimits.StatorCurrentLimitEnable= true;
        deployConfig.CurrentLimits.StatorCurrentLimit= ClimberConstants.CurrentLimits.currentLimit;

        deployConfig.Slot0.kP= ClimberConstants.PID.deployKP;
        deployConfig.Slot0.kI= ClimberConstants.PID.deployKI;
        deployConfig.Slot0.kD= ClimberConstants.PID.deployKD;
        deployConfig.Slot0.kS= ClimberConstants.PID.deployKS;
        deployConfig.Slot0.kV= ClimberConstants.PID.deployKV;
        deployConfig.Slot0.kG= ClimberConstants.PID.deployKG;

        deployConfig.MotionMagic.MotionMagicCruiseVelocity= ClimberConstants.MotionMagic.deployCruiseVelocity.in(DegreesPerSecond);
        deployConfig.MotionMagic.MotionMagicAcceleration= ClimberConstants.MotionMagic.deployAngularAcceleration.in(DegreesPerSecondPerSecond);
        deployConfig.MotionMagic.MotionMagicJerk= ClimberConstants.MotionMagic.deployJerk;

        deployConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable= true;
        deployConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold= ClimberConstants.AngleSetpoints.deployMaxAngle.in(Degrees);
        deployConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable= true;
        deployConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold= ClimberConstants.AngleSetpoints.deployMinAngle.in(Degrees);

        tryUntilOk(5, () -> deployMotor.getConfigurator().apply(deployConfig, 0.25));

        TalonFXConfiguration twistConfig= new TalonFXConfiguration();

        twistConfig.MotorOutput.NeutralMode= NeutralModeValue.Brake;

        twistConfig.CurrentLimits.StatorCurrentLimitEnable= true;
        twistConfig.CurrentLimits.StatorCurrentLimit= ClimberConstants.CurrentLimits.currentLimit;

        twistConfig.Slot0.kP= ClimberConstants.PID.twistKP;
        twistConfig.Slot0.kI= ClimberConstants.PID.twistKI;
        twistConfig.Slot0.kD= ClimberConstants.PID.twistKD;
        twistConfig.Slot0.kS= ClimberConstants.PID.twistKS;
        twistConfig.Slot0.kV= ClimberConstants.PID.twistKV;
        twistConfig.Slot0.kG= ClimberConstants.PID.twistKG;

        twistConfig.MotionMagic.MotionMagicCruiseVelocity= ClimberConstants.MotionMagic.twistCruiseVelocity.in(DegreesPerSecond);
        twistConfig.MotionMagic.MotionMagicAcceleration= ClimberConstants.MotionMagic.twistAngularAcceleration.in(DegreesPerSecondPerSecond);
        twistConfig.MotionMagic.MotionMagicJerk= ClimberConstants.MotionMagic.twistJerk;

        twistConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable= true;
        twistConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold= ClimberConstants.AngleSetpoints.twistMaxAngle.in(Degrees);
        twistConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable= true;
        twistConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold= ClimberConstants.AngleSetpoints.twistMinAngle.in(Degrees);

        tryUntilOk(5, () -> twistMotor.getConfigurator().apply(twistConfig, 0.25));

        deployPositionSig= deployMotor.getPosition();
        deployVoltageSig= deployMotor.getMotorVoltage();
        deployCurrentSig= deployMotor.getStatorCurrent();
        deployVelocitySig= deployMotor.getVelocity();

        twistPositionSig= twistMotor.getPosition();
        twistVoltageSig= twistMotor.getMotorVoltage();
        twistCurrentSig= twistMotor.getStatorCurrent();
        twistVelocitySig= twistMotor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(20, deployVoltageSig, deployCurrentSig, deployVelocitySig, twistVoltageSig, twistCurrentSig, twistVelocitySig);
        BaseStatusSignal.setUpdateFrequencyForAll(50, deployPositionSig, twistPositionSig);

        ParentDevice.optimizeBusUtilizationForAll(deployMotor, twistMotor);

        @Override

        public void updateInputs(ClimberIOInputs inputs) {
            BaseStatusSignal.refreshAll(
                deployPositionSig,
                deployVoltageSig,
                deployCurrentSig,
                deployVelocitySig,
                twistPositionSig,
                twistVoltageSig,
                twistCurrentSig,
                twistVelocitySig
            );

            inputs.deployPosition= deployPositionSig.getValue();
            inputs.deployVolts= deployVoltageSig.getValue();
            inputs.deployCurrent= deployCurrentSig.getValue();
            inputs.deployVelocity= deployVelocitySig.getValue();

            inputs.twistPosition= twistPositionSig.getValue();
            inputs.twistVolts= twistVoltageSig.getValue();
            inputs.twistCurrent= twistCurrentSig.getValue();
            inputs.twistVelocity= twistVelocitySig.getValue();
        }
        
        @Override
        public void setDeployAngle(Angle angle) {
            deployMotor.setControl(deployAngleRequest.withPosition(angle));
        }

        @Override
        public void setDeployVelocity(AngularVelocity velocity) {
            deployMotor.setControl(deployVelocityRequest.withVelocity(velocity));
        }

        @Override
        public void setTwistAngle(Angle angle) {
            twistMotor.setControl(twistAngleRequest.withPosition(angle));
        }

        @Override
        public void setTwistVelocity(AngularVelocity velocity) {
            twistMotor.setControl(twistVelocityRequest.withVelocity(velocity));
        }

        @Override
        public void setTwistVoltage(Voltage voltage) {
            twistMotor.setControl(twistVoltageRequest.withOutput(voltage));
        }

        @Override
        public void stopTwist() {
            twistMotor.setControl(twistVoltageRequest.withOutput(Volts.of(0)));
        }

        @Override
        public void stopDeploy() {
            deployMotor.setControl(deployVoltageRequest.withOutput(Volts.of(0)));
        }
    }
}