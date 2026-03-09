package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FlywheelIOSim implements FlywheelIO {
    private DCMotorSim sim;

    private SimpleMotorFeedforward feedforward;

    private MutVoltage cachedVoltage = Units.Volts.mutable(0.0);

    public FlywheelIOSim(
            double gearing,
            MomentOfInertia moi) {
        sim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60Foc(1),
                        moi.in(Units.KilogramSquareMeters),
                        gearing),
                DCMotor.getKrakenX60Foc(1));

        feedforward = new SimpleMotorFeedforward(0.0, 0.116379);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        sim.setInputVoltage(cachedVoltage.in(Units.Volts));

        sim.update(Constants.LOOP_TIME);

        inputs.isConnected = true;
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
        cachedVoltage.mut_replace(voltage);
    }

    @Override
    public void requestAngularVelocity(AngularVelocity velocity) {
        cachedVoltage.mut_replace(
            feedforward.calculate(velocity.in(Units.RotationsPerSecond)),
            Units.Volts
        );
    }

    @Override
    public void requestLinearVelocity(LinearVelocity velocity) {
        requestAngularVelocity(
                Units.RadiansPerSecond
                        .of(velocity.in(Units.MetersPerSecond) / FlywheelConstants.FLYWHEEL_RADIUS.in(Units.Meters)));
    }

    @Override
    public void stop() {
        requestVoltage(Units.Volts.zero());
    }

}
