package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO {
    
    protected TalonFX shooter1Motor;
    protected TalonFX shooter2Motor;
    protected TalonFX shooter3Motor;

    private StatusSignalCollection signals;

    private StatusSignal<AngularVelocity> shooter1AngularVelocity;
    private StatusSignal<Voltage> shooter1AppliedVolts;
    private StatusSignal<Current> shooter1CurrentAmps;
    private StatusSignal<Temperature> shooter1Temp;

    private StatusSignal<AngularVelocity> shooter2AngularVelocity;
    private StatusSignal<Voltage> shooter2AppliedVolts;
    private StatusSignal<Current> shooter2CurrentAmps;
    private StatusSignal<Temperature> shooter2Temp;

    private StatusSignal<AngularVelocity> shooter3AngularVelocity;
    private StatusSignal<Voltage> shooter3AppliedVolts;
    private StatusSignal<Current> shooter3CurrentAmps;
    private StatusSignal<Temperature> shooter3Temp;

    public ShooterIOTalonFX(CANBus shooterCANBus) {

        shooter1Motor = new TalonFX(ShooterConstants.shooter1CANID, shooterCANBus);
        shooter2Motor = new TalonFX(ShooterConstants.shooter2CANID, shooterCANBus);
        shooter3Motor = new TalonFX(ShooterConstants.shooter3CANID, shooterCANBus);

        final TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();

        // PID 
        shooterConfigs.Slot0.kP = ShooterConstants.PID.shooterkP;
        shooterConfigs.Slot0.kD = ShooterConstants.PID.shooterkD;
        shooterConfigs.Slot0.kI = ShooterConstants.PID.shooterkI;
        shooterConfigs.Slot0.kV = ShooterConstants.PID.shooterkV;
        shooterConfigs.Slot0.kA = ShooterConstants.PID.shooterkA;
        shooterConfigs.Slot0.kG = ShooterConstants.PID.shooterkG;
        shooterConfigs.Slot0.kS = ShooterConstants.PID.shooterkS;

        // Motion Magic Configurations
        shooterConfigs.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.MotionMagic.shooterkMaxVelocity; 
        shooterConfigs.MotionMagic.MotionMagicAcceleration = ShooterConstants.MotionMagic.shooterkMaxAcceleration; 
        shooterConfigs.MotionMagic.MotionMagicJerk = ShooterConstants.MotionMagic.shooterkJerk;

        // Current Limits
        shooterConfigs.CurrentLimits.SupplyCurrentLimit = ShooterConstants.currentLimit.in(Amps);
        shooterConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        shooterConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive ;

        // Similar to our checkError function
        tryUntilOk(5, () -> shooter1Motor.getConfigurator().apply(shooterConfigs, 0.25));
        tryUntilOk(5, () -> shooter2Motor.getConfigurator().apply(shooterConfigs, 0.25));
        tryUntilOk(5, () -> shooter3Motor.getConfigurator().apply(shooterConfigs, 0.25));

        // Setting other shooter motors as followers
        shooter2Motor.setControl(new Follower(ShooterConstants.shooter1CANID, MotorAlignmentValue.Aligned));
        shooter3Motor.setControl(new Follower(ShooterConstants.shooter1CANID, MotorAlignmentValue.Aligned));

        shooter1AngularVelocity = shooter1Motor.getVelocity();
        shooter1AppliedVolts = shooter1Motor.getMotorVoltage();
        shooter1CurrentAmps = shooter1Motor.getSupplyCurrent();
        shooter1Temp = shooter1Motor.getDeviceTemp();
        shooter2AngularVelocity = shooter2Motor.getVelocity();
        shooter2AppliedVolts = shooter2Motor.getMotorVoltage();
        shooter2CurrentAmps = shooter2Motor.getSupplyCurrent();
        shooter2Temp = shooter2Motor.getDeviceTemp();
        shooter3AngularVelocity = shooter3Motor.getVelocity();
        shooter3AppliedVolts = shooter3Motor.getMotorVoltage();
        shooter3CurrentAmps = shooter3Motor.getSupplyCurrent();
        shooter3Temp = shooter3Motor.getDeviceTemp();
        
        // Status Signal Collection, less repetitive code
        signals = new StatusSignalCollection(
            shooter1AngularVelocity,
            shooter1AppliedVolts,
            shooter1CurrentAmps,
            shooter1Temp,
            shooter2AngularVelocity,
            shooter2AppliedVolts,
            shooter2CurrentAmps,
            shooter2Temp,
            shooter3AngularVelocity,
            shooter3AppliedVolts,
            shooter3CurrentAmps,
            shooter3Temp
        );

        tryUntilOk(5, () -> signals.setUpdateFrequencyForAll(50.0));

        ParentDevice.optimizeBusUtilizationForAll(shooter1Motor, shooter2Motor, shooter3Motor);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        var status = signals.refreshAll();

        inputs.allConnected = status.isOK();

        inputs.shooter1Velocity = shooter1AngularVelocity.getValue();
        inputs.shooter2Velocity = shooter2AngularVelocity.getValue();
        inputs.shooter3Velocity = shooter3AngularVelocity.getValue();

        inputs.shooter1Voltage = shooter1AppliedVolts.getValue();
        inputs.shooter2Voltage = shooter2AppliedVolts.getValue();
        inputs.shooter3Voltage = shooter3AppliedVolts.getValue();

        inputs.shooter1Current = shooter1CurrentAmps.getValue();
        inputs.shooter2Current = shooter2CurrentAmps.getValue();
        inputs.shooter3Current = shooter3CurrentAmps.getValue();

        inputs.shooter1Temp = shooter1Temp.getValue();
        inputs.shooter2Temp = shooter2Temp.getValue();
        inputs.shooter3Temp = shooter3Temp.getValue();

        inputs.wheelVelocity = inputs.shooter1Velocity.times(ShooterConstants.gearRatio);
    }

    public void setVelocity(AngularVelocity vel) {
        AngularVelocity velocity = vel.div(ShooterConstants.gearRatio);
        if (vel.lt(ShooterConstants.minimumVelocitySetpoint)) {
            setVoltage(Volts.zero());
            return;
        }
        shooter1Motor.setControl(new VelocityVoltage(velocity).withEnableFOC(true));
    }

    public void setVoltage(Voltage vol) {
        shooter1Motor.setControl(new VoltageOut(vol));
    }

    public void stop() {
        shooter1Motor.stopMotor();
    }
}
