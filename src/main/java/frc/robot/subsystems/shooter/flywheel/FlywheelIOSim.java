package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FlywheelIOSim implements FlywheelIO {
    private DCMotorSim sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(1),
                    0.0007528205,
                    FlywheelConstants.SHOOTER_GEARING),
            DCMotor.getKrakenX60Foc(1));

    private PIDController pid = new PIDController(FlywheelConstants.LEFT_PID.kP(), FlywheelConstants.LEFT_PID.kI(),
            FlywheelConstants.LEFT_PID.kD());
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(FlywheelConstants.LEFT_FF.kS(),
            FlywheelConstants.LEFT_FF.kV());

    private MutVoltage cachedVoltage = Units.Volts.mutable(0.0);
    private boolean isOpenLoop = false;

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        if (isOpenLoop) {
            pid.reset();
            pid.setSetpoint(0.0);
            sim.setInputVoltage(cachedVoltage.in(Units.Volts));
        } else {
            cachedVoltage.mut_replace(pid.calculate(sim.getAngularVelocity().in(Units.RotationsPerSecond)),
                    Units.Volts);
            sim.setInputVoltage(cachedVoltage.in(Units.Volts));
        }

        sim.update(Constants.LOOP_TIME);

        inputs.appliedVoltage.mut_replace(cachedVoltage);
        inputs.angularPosition.mut_replace(sim.getAngularPosition());
        inputs.angularVelocity.mut_replace(sim.getAngularVelocity());
        inputs.linearVelocity.mut_replace(
                inputs.angularVelocity.baseUnitMagnitude() * FlywheelConstants.FLYWHEEL_RADIUS.baseUnitMagnitude(),
                Units.MetersPerSecond);
        inputs.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Units.Amps);
        inputs.statorCurrent.mut_replace(inputs.supplyCurrent);
    }

    @Override
    public void requestVoltage(Voltage voltage) {
        isOpenLoop = true;
        cachedVoltage.mut_replace(voltage);
    }
    @Override
    public void requestVelocity(AngularVelocity velocity) {
        isOpenLoop = false;
        pid.setSetpoint(velocity.in(Units.RotationsPerSecond));
    }

    @Override
    public  void stop() {
        requestVoltage(Units.Volts.zero());
    }

}
