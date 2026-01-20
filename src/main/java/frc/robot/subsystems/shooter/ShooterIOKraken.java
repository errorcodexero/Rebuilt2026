package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.*;

public class ShooterIOKraken implements ShooterIO {
  private final TalonFX[] flywheelMotors;
  private static final int NUM_MOTORS = ShooterConstants.FLYWHEEL_MOTOR_CAN_IDS.length;

  // Control requests
  private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  // Status signals - Arrays for all motors
  private final StatusSignal<AngularVelocity>[] flywheelVelocity;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage>[] flywheelAppliedVolts;
  private final StatusSignal<edu.wpi.first.units.measure.Current>[] flywheelCurrent;
  private final StatusSignal<edu.wpi.first.units.measure.Temperature>[] flywheelTemperature;

  private DCMotorSim flywheelSim;

  @SuppressWarnings("unchecked")
  public ShooterIOKraken() {
    // Initialize motor array
    flywheelMotors = new TalonFX[NUM_MOTORS];
    for (int i = 0; i < NUM_MOTORS; i++) {
      flywheelMotors[i] = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR_CAN_IDS[i]);
    }

    // Initialize status signal arrays
    flywheelVelocity = new StatusSignal[NUM_MOTORS];
    flywheelAppliedVolts = new StatusSignal[NUM_MOTORS];
    flywheelCurrent = new StatusSignal[NUM_MOTORS];
    flywheelTemperature = new StatusSignal[NUM_MOTORS];

    // Configure leader motor (index 0)
    var leaderConfig = new TalonFXConfiguration();
    
    // Motor output configuration
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    
    // Current limits
    leaderConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.FLYWHEEL_CURRENT_LIMIT.in(Amps);
    leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    
    // Feedback configuration
    leaderConfig.Feedback.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_GEAR_RATIO;
    
    // PID configuration for velocity control
    leaderConfig.Slot0.kP = ShooterConstants.FLYWHEEL_KP;
    leaderConfig.Slot0.kI = ShooterConstants.FLYWHEEL_KI;
    leaderConfig.Slot0.kD = ShooterConstants.FLYWHEEL_KD;
    leaderConfig.Slot0.kS = ShooterConstants.FLYWHEEL_KS;
    leaderConfig.Slot0.kV = ShooterConstants.FLYWHEEL_KV;
    leaderConfig.Slot0.kA = ShooterConstants.FLYWHEEL_KA;
    
    tryUntilOk(5, () -> flywheelMotors[0].getConfigurator().apply(leaderConfig, 0.25));
    
    // Configure follower motors (index 1 and higher)
    var followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Match leader direction
    followerConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.FLYWHEEL_CURRENT_LIMIT.in(Amps);
    followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    
    for (int i = 1; i < NUM_MOTORS; i++) {
      final int motorIndex = i; // Make effectively final for lambda
      tryUntilOk(5, () -> flywheelMotors[motorIndex].getConfigurator().apply(followerConfig, 0.25));
      // Set followers to follow leader (motor 0)
      flywheelMotors[i].setControl(new com.ctre.phoenix6.controls.StrictFollower(ShooterConstants.FLYWHEEL_MOTOR_CAN_IDS[0]));
    }
    
    // Initialize status signals for all motors
    for (int i = 0; i < NUM_MOTORS; i++) {
      flywheelVelocity[i] = flywheelMotors[i].getVelocity();
      flywheelAppliedVolts[i] = flywheelMotors[i].getMotorVoltage();
      flywheelCurrent[i] = flywheelMotors[i].getStatorCurrent();
      flywheelTemperature[i] = flywheelMotors[i].getDeviceTemp();
    }
    
    // Set update frequencies - build arrays for all signals
    StatusSignal<?>[] highFreqSignals = new StatusSignal<?>[NUM_MOTORS * 3]; // velocity, voltage, current
    StatusSignal<?>[] lowFreqSignals = new StatusSignal<?>[NUM_MOTORS]; // temperature
    
    for (int i = 0; i < NUM_MOTORS; i++) {
      highFreqSignals[i * 3] = flywheelVelocity[i];
      highFreqSignals[i * 3 + 1] = flywheelAppliedVolts[i];
      highFreqSignals[i * 3 + 2] = flywheelCurrent[i];
      lowFreqSignals[i] = flywheelTemperature[i];
    }
    
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, highFreqSignals); // 50 Hz
    BaseStatusSignal.setUpdateFrequencyForAll(4.0, lowFreqSignals); // 4 Hz for temperature
    
    // Optimize bus utilization for all motors
    for (int i = 0; i < NUM_MOTORS; i++) {
      flywheelMotors[i].optimizeBusUtilization();
    }

    if (RobotBase.isSimulation()) {
      // Flywheel uses 3 motors mechanically coupled
      DCMotor flywheelGearbox = DCMotor.getKrakenX60(ShooterConstants.FLYWHEEL_MOTOR_CAN_IDS.length);
      flywheelSim = new DCMotorSim(
          edu.wpi.first.math.system.plant.LinearSystemId.createDCMotorSystem(
              flywheelGearbox,
              ShooterConstants.FLYWHEEL_MOMENT_OF_INERTIA.in(KilogramSquareMeters),
              ShooterConstants.FLYWHEEL_GEAR_RATIO
          ),
          flywheelGearbox
      );
    }
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Refresh all signals
    StatusSignal<?>[] allSignals = new StatusSignal<?>[NUM_MOTORS * 4];
    for (int i = 0; i < NUM_MOTORS; i++) {
      allSignals[i * 4] = flywheelVelocity[i];
      allSignals[i * 4 + 1] = flywheelAppliedVolts[i];
      allSignals[i * 4 + 2] = flywheelCurrent[i];
      allSignals[i * 4 + 3] = flywheelTemperature[i];
    }
    BaseStatusSignal.refreshAll(allSignals);
    
    // Update flywheel inputs
    inputs.flywheelVelocity = flywheelVelocity[0].getValue(); // Leader velocity
    inputs.flywheelAppliedVoltage = flywheelAppliedVolts[0].getValue(); // Leader voltage only
    
    // Sum current from all motors and populate individual currents
    double totalCurrentAmps = 0.0;
    for (int i = 0; i < NUM_MOTORS; i++) {
      totalCurrentAmps += flywheelCurrent[i].getValue().in(Amps);
    }
    inputs.flywheelCurrent0 = flywheelCurrent[0].getValue();
    inputs.flywheelCurrent1 = flywheelCurrent[1].getValue();
    inputs.flywheelCurrent2 = flywheelCurrent[2].getValue();
    inputs.flywheelTotalCurrent = Amps.of(totalCurrentAmps);
    
    // Find maximum temperature across all motors
    double maxTempCelsius = flywheelTemperature[0].getValue().in(Celsius);
    for (int i = 1; i < NUM_MOTORS; i++) {
      double temp = flywheelTemperature[i].getValue().in(Celsius);
      if (temp > maxTempCelsius) {
        maxTempCelsius = temp;
      }
    }
    inputs.flywheelMaxTemperature = maxTempCelsius;

    if (Robot.isSimulation()) {
      // Update flywheel simulation
      flywheelSim.update(0.020); // 20ms robot loop

      TalonFXSimState st = flywheelMotors[0].getSimState() ;
      st.setSupplyVoltage(RobotController.getBatteryVoltage()) ;
   
      // Otherwise use the voltage set directly via setFlywheelVoltage()   
      flywheelSim.setInputVoltage(st.getMotorVoltage()) ;
      
      // Update inputs - Flywheels
      inputs.flywheelVelocity = RotationsPerSecond.of(flywheelSim.getAngularVelocityRPM() / 60.0);
      inputs.flywheelAppliedVoltage = st.getMotorVoltageMeasure() ;
      
      // Split total current among motors for simulation
      double totalCurrent = Math.abs(flywheelSim.getCurrentDrawAmps());
      inputs.flywheelTotalCurrent = Amps.of(totalCurrent);
      double currentPerMotor = totalCurrent / NUM_MOTORS;
      
      inputs.flywheelMaxTemperature = 25.0; // Simulated constant temperature
    }
  }

  @Override
  public void setFlywheelVelocity(AngularVelocity velocity) {
    flywheelMotors[0].setControl(
        flywheelVelocityRequest
            .withVelocity(velocity)
            .withSlot(0)
    );
    // Followers automatically follow
  }

  @Override
  public void setFlywheelVoltage(edu.wpi.first.units.measure.Voltage voltage) {
    flywheelMotors[0].setControl(voltageRequest.withOutput(voltage.in(Volts)));
    // Followers automatically follow
  }

  @Override
  public void stopFlywheels() {
    flywheelMotors[0].setControl(voltageRequest.withOutput(0.0));
    // Followers automatically follow
  }

  @Override
  public void setFlywheelBrake(boolean brake) {
    for (int i = 0; i < NUM_MOTORS; i++) {
      flywheelMotors[i].setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
  }
}
