package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.CompTunerConstants;

public class ShooterIOHardware implements ShooterIO {
    
    private TalonFX shooter1Motor;
    private TalonFX shooter2Motor;
    private TalonFX shooter3Motor;
    private TalonFX shooter4Motor;
    
    private TalonFX hoodMotor;


    //Pivot status signals
    private StatusSignal<Angle> hoodAngleSignal;
    private StatusSignal<AngularVelocity> hoodAngularVelocitySignal;
    private StatusSignal<Current> hoodCurrentAmpsSignal;
    private StatusSignal<Voltage> hoodAppliedVoltsSignal;

    
    private StatusSignal<AngularVelocity> shooter1AngularVelocitySignal;
    private StatusSignal<Voltage> shooter1AppliedVoltsSignal;
    private StatusSignal<Current> shooter1CurrentAmpsSignal;

    private StatusSignal<AngularVelocity> shooter2AngularVelocitySignal;
    private StatusSignal<Voltage> shooter2AppliedVoltsSignal;
    private StatusSignal<Current> shooter2CurrentAmpsSignal;

    private StatusSignal<AngularVelocity> shooter3AngularVelocitySignal;
    private StatusSignal<Voltage> shooter3AppliedVoltsSignal;
    private StatusSignal<Current> shooter3CurrentAmpsSignal;

    private StatusSignal<AngularVelocity> shooter4AngularVelocitySignal;
    private StatusSignal<Voltage> shooter4AppliedVoltsSignal;
    private StatusSignal<Current> shooter4CurrentAmpsSignal;

    public ShooterIOHardware() {

        shooter1Motor = new TalonFX(ShooterConstants.shooter1CANID, CompTunerConstants.kCANBus);
        shooter2Motor = new TalonFX(ShooterConstants.shooter2CANID, CompTunerConstants.kCANBus);
        shooter3Motor = new TalonFX(ShooterConstants.shooter3CANID, CompTunerConstants.kCANBus);
        shooter4Motor = new TalonFX(ShooterConstants.shooter4CANID, CompTunerConstants.kCANBus);
        
        hoodMotor = new TalonFX(ShooterConstants.hoodCANID, CompTunerConstants.kCANBus);

        final TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();
        final TalonFXConfiguration hoodConfigs = new TalonFXConfiguration();

        // PID 
        shooterConfigs.Slot0.kP = ShooterConstants.PID.shooterkP;
        shooterConfigs.Slot0.kD = ShooterConstants.PID.shooterkD;
        shooterConfigs.Slot0.kI = ShooterConstants.PID.shooterkI;
        shooterConfigs.Slot0.kV = ShooterConstants.PID.shooterkV;
        shooterConfigs.Slot0.kA = ShooterConstants.PID.shooterkA;
        shooterConfigs.Slot0.kG = ShooterConstants.PID.shooterkG;
        shooterConfigs.Slot0.kS = ShooterConstants.PID.shooterkS;

        shooter1Motor.getConfigurator().apply(shooterConfigs);
        shooter2Motor.getConfigurator().apply(shooterConfigs);
        shooter3Motor.getConfigurator().apply(shooterConfigs);
        shooter4Motor.getConfigurator().apply(shooterConfigs);


        hoodConfigs.Slot0.kP = ShooterConstants.PID.hoodkP;
        hoodConfigs.Slot0.kD = ShooterConstants.PID.hoodkD;
        hoodConfigs.Slot0.kI = ShooterConstants.PID.hoodkI;
        hoodConfigs.Slot0.kV = ShooterConstants.PID.hoodkV;
        hoodConfigs.Slot0.kA = ShooterConstants.PID.hoodkA;
        hoodConfigs.Slot0.kG = ShooterConstants.PID.hoodkG;
        hoodConfigs.Slot0.kS = ShooterConstants.PID.hoodkS;


        // Software Limits - only hood
        hoodConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ShooterConstants.SoftwareLimits.hoodMaxAngle;
        hoodConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ShooterConstants.SoftwareLimits.hoodMinAngle;

        // Motion Magic Configurations
        shooterConfigs.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.MotionMagic.shooterkMaxVelocity; 
        shooterConfigs.MotionMagic.MotionMagicAcceleration = ShooterConstants.MotionMagic.shooterkMaxAcceleration; 
        shooterConfigs.MotionMagic.MotionMagicJerk = ShooterConstants.MotionMagic.shooterkJerk;

        // Current Limits
        shooterConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.currentLimit;
        shooterConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        hoodConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.currentLimit;
        shooterConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        // Similar to our checkError function
        tryUntilOk(5, () -> shooter1Motor.getConfigurator().apply(shooterConfigs, 0.25));
        tryUntilOk(5, () -> shooter2Motor.getConfigurator().apply(shooterConfigs, 0.25));
        tryUntilOk(5, () -> shooter3Motor.getConfigurator().apply(shooterConfigs, 0.25));
        tryUntilOk(5, () -> shooter4Motor.getConfigurator().apply(shooterConfigs, 0.25));
        tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfigs, 0.25));

        // Setting other shooter motors as followers
        shooter2Motor.setControl(new Follower(ShooterConstants.shooter1CANID, null));
        shooter3Motor.setControl(new Follower(ShooterConstants.shooter1CANID, null));
        shooter4Motor.setControl(new Follower(ShooterConstants.shooter1CANID, null));


        // Status Signal Collection, less repetitive code
        StatusSignalCollection signals = new StatusSignalCollection(
        hoodAngleSignal = hoodMotor.getPosition(),
        hoodAngularVelocitySignal = hoodMotor.getVelocity(),
        hoodAppliedVoltsSignal = hoodMotor.getSupplyVoltage(),
        hoodCurrentAmpsSignal = hoodMotor.getSupplyCurrent(),
        shooter1AngularVelocitySignal = shooter1Motor.getVelocity(),
        shooter1AppliedVoltsSignal = shooter1Motor.getSupplyVoltage(),
        shooter1CurrentAmpsSignal = shooter1Motor.getSupplyCurrent(),
        shooter2AngularVelocitySignal = shooter2Motor.getVelocity(),
        shooter2AppliedVoltsSignal = shooter2Motor.getSupplyVoltage(),
        shooter2CurrentAmpsSignal = shooter2Motor.getSupplyCurrent(),
        shooter3AngularVelocitySignal = shooter3Motor.getVelocity(),
        shooter3AppliedVoltsSignal = shooter3Motor.getSupplyVoltage(),
        shooter3CurrentAmpsSignal = shooter3Motor.getSupplyCurrent(),
        shooter4AngularVelocitySignal = shooter4Motor.getVelocity(),
        shooter4AppliedVoltsSignal = shooter4Motor.getSupplyVoltage(),
        shooter4CurrentAmpsSignal = shooter4Motor.getSupplyCurrent()
        );

        signals.setUpdateFrequencyForAll(50.0);

        ParentDevice.optimizeBusUtilizationForAll(shooter1Motor, shooter2Motor, shooter3Motor, shooter4Motor, hoodMotor);
    }
    
    public void updateInputs(ShooterIOInputsAutoLogged inputs, StatusSignalCollection signals) {
        signals.refreshAll();

        inputs.hoodPosition = hoodAngleSignal.getValue();
        inputs.hoodVelocity = hoodAngularVelocitySignal.getValue();
        inputs.hoodCurrent = hoodCurrentAmpsSignal.getValue();
        inputs.hoodVoltage = hoodAppliedVoltsSignal.getValue();

        inputs.shooter1Velocity = shooter1AngularVelocitySignal.getValue();
        inputs.shooter2Velocity = shooter2AngularVelocitySignal.getValue();
        inputs.shooter3Velocity = shooter3AngularVelocitySignal.getValue();
        inputs.shooter4Velocity = shooter4AngularVelocitySignal.getValue();

        inputs.shooter1Voltage = shooter1AppliedVoltsSignal.getValue();
        inputs.shooter2Voltage = shooter2AppliedVoltsSignal.getValue();
        inputs.shooter3Voltage = shooter3AppliedVoltsSignal.getValue();
        inputs.shooter4Voltage = shooter4AppliedVoltsSignal.getValue();

        inputs.shooter1Current = shooter1CurrentAmpsSignal.getValue();
        inputs.shooter2Current = shooter2CurrentAmpsSignal.getValue();
        inputs.shooter3Current = shooter3CurrentAmpsSignal.getValue();
        inputs.shooter4Current = shooter4CurrentAmpsSignal.getValue();
    }

    public void setShooterVelocity(AngularVelocity vel) {
        AngularVelocity velocity = vel.div(ShooterConstants.gearRatio);
        shooter1Motor.setControl(new MotionMagicVelocityVoltage(velocity));
    }

    public void setShooterVoltage(Voltage vol) {
        shooter1Motor.setControl(new VoltageOut(vol));
    }

    public void stopShooter() {
        shooter1Motor.set(0);
    }

    public void setHoodPosition(Angle pos) {
        shooter1Motor.setControl(new MotionMagicVoltage(pos).withEnableFOC(true));
    }
}
