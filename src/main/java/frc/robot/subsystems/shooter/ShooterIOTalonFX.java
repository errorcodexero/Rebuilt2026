package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;

public class ShooterIOTalonFX implements ShooterIO {
    
    private TalonFX shooter1Motor;
    private TalonFX shooter2Motor;
    private TalonFX shooter3Motor;

    private Servo hood1;
    private Servo hood2;
    

    private StatusSignalCollection signals;

    private StatusSignal<AngularVelocity> shooter1AngularVelocity;
    private StatusSignal<Voltage> shooter1AppliedVolts;
    private StatusSignal<Current> shooter1CurrentAmps;

    private StatusSignal<AngularVelocity> shooter2AngularVelocity;
    private StatusSignal<Voltage> shooter2AppliedVolts;
    private StatusSignal<Current> shooter2CurrentAmps;

    private StatusSignal<AngularVelocity> shooter3AngularVelocity;
    private StatusSignal<Voltage> shooter3AppliedVolts;
    private StatusSignal<Current> shooter3CurrentAmps;

    public ShooterIOTalonFX(CANBus shooterCANBus) {

        shooter1Motor = new TalonFX(ShooterConstants.shooter1CANID, shooterCANBus);
        shooter2Motor = new TalonFX(ShooterConstants.shooter2CANID, shooterCANBus);
        shooter3Motor = new TalonFX(ShooterConstants.shooter3CANID, shooterCANBus);
        //shooter4Motor = new TalonFX(ShooterConstants.shooter4CANID, shooterCANBus);
        
        //hoodMotor = new TalonFX(ShooterConstants.hoodCANID, shooterCANBus);
        hood1 = new Servo(0);
        hood2 = new Servo(1);

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

        // Setting other shooter motors as followers
        shooter2Motor.setControl(new Follower(ShooterConstants.shooter1CANID, null));
        shooter3Motor.setControl(new Follower(ShooterConstants.shooter1CANID, null));


        shooter1AngularVelocity = shooter1Motor.getVelocity();
        shooter1AppliedVolts = shooter1Motor.getSupplyVoltage();
        shooter1CurrentAmps = shooter1Motor.getSupplyCurrent();
        shooter2AngularVelocity = shooter2Motor.getVelocity();
        shooter2AppliedVolts = shooter2Motor.getSupplyVoltage();
        shooter2CurrentAmps = shooter2Motor.getSupplyCurrent();
        shooter3AngularVelocity = shooter3Motor.getVelocity();
        shooter3AppliedVolts = shooter3Motor.getSupplyVoltage();
        shooter3CurrentAmps = shooter3Motor.getSupplyCurrent();
        
        // Status Signal Collection, less repetitive code
        StatusSignalCollection signals = new StatusSignalCollection(
            shooter1AngularVelocity,
            shooter1AppliedVolts,
            shooter1CurrentAmps,
            shooter2AngularVelocity,
            shooter2AppliedVolts,
            shooter2CurrentAmps,
            shooter3AngularVelocity,
            shooter3AppliedVolts,
            shooter3CurrentAmps
        );

        tryUntilOk(5, () -> signals.setUpdateFrequencyForAll(50.0));

        ParentDevice.optimizeBusUtilizationForAll(shooter1Motor, shooter2Motor, shooter3Motor);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        signals.refreshAll();

        inputs.shooter1Velocity = shooter1AngularVelocity.getValue();
        inputs.shooter2Velocity = shooter2AngularVelocity.getValue();
        inputs.shooter3Velocity = shooter3AngularVelocity.getValue();

        inputs.shooter1Voltage = shooter1AppliedVolts.getValue();
        inputs.shooter2Voltage = shooter2AppliedVolts.getValue();
        inputs.shooter3Voltage = shooter3AppliedVolts.getValue();

        inputs.shooter1Current = shooter1CurrentAmps.getValue();
        inputs.shooter2Current = shooter2CurrentAmps.getValue();
        inputs.shooter3Current = shooter3CurrentAmps.getValue();
    }

    public void setShooterVelocity(AngularVelocity vel) {
        AngularVelocity velocity = vel.times(ShooterConstants.gearRatio);
        shooter1Motor.setControl(new MotionMagicVelocityVoltage(velocity));
    }

    public void setShooterVoltage(Voltage vol) {
        shooter1Motor.setControl(new VoltageOut(vol));
    }

    public void stopShooter() {
        shooter1Motor.set(0);
    }

    public void setHoodPosition(Angle pos) {
        double position = pos.in(Degrees);
        hood1.setAngle(position);
        hood2.setAngle(position);
    }
}
