package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import sun.security.provider.DSAKeyPairGenerator.Current;

public class HoodIOSim implements HoodIO {
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                    DCMotor.getKrakenX44Foc(1),
                    0.0342069049,
                    HoodConstants.HOOD_GEARING),
            DCMotor.getKrakenX44Foc(1),
            HoodConstants.HOOD_GEARING,
            0.23114,
            HoodConstants.MIN_ANGLE.in(Units.Radians),
            HoodConstants.MAX_ANGLE.in(Units.Radians),
            true,
            0.0);

    private PIDController pid = new PIDController(
            HoodConstants.PID_CONSTANTS.kP(),
            HoodConstants.PID_CONSTANTS.kI(),
            HoodConstants.PID_CONSTANTS.kD());

    private ArmFeedforward ff = new ArmFeedforward(
            HoodConstants.FF_CONSTANTS.kS(),
            HoodConstants.FF_CONSTANTS.kG(),
            HoodConstants.FF_CONSTANTS.kV(),
            HoodConstants.FF_CONSTANTS.kA());

    private MutVoltage cachedVoltage = Units.Volts.mutable(0.0);
    private boolean isOpenLoop = true;

    @Override
    public void updateInputs(HoodIOInputs inputs) {

        if (isOpenLoop) {
            pid.reset();
            pid.setSetpoint(0.0);
            sim.setInputVoltage(cachedVoltage.in(Units.Volts));
        } else {
            cachedVoltage.mut_replace(pid.calculate(sim.getAngleRads() / (2 * Math.PI)),
                    Units.Volts);
            sim.setInputVoltage(cachedVoltage.in(Units.Volts));
        }

        sim.update(Constants.LOOP_TIME);

        inputs.isConnected = true;
        inputs.appliedVoltage.mut_replace(cachedVoltage);
        inputs.angularPosition.mut_replace(sim.getAngleRads(), Units.Radians);
        inputs.angularVelocity.mut_replace(sim.getVelocityRadPerSec(), Units.RadiansPerSecond);
        inputs.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Units.Amps);
        inputs.statorCurrent.mut_replace(inputs.supplyCurrent);
    }

    @Override
    public void requestVoltage(Voltage voltage) {
        isOpenLoop = true;
        cachedVoltage.mut_replace(voltage);
    }

    @Override
    public void requestPosition(Angle position) {
        isOpenLoop = false;
        pid.setSetpoint(position.in(Units.Rotations));
    }

    @Override
    public void setEncoderPosition(Angle position) {
        double currentVelocity = sim.getVelocityRadPerSec();
        sim.setState(position.in(Units.Radians), currentVelocity);
    }

    @Override
    public void resetEncoderPosition() {
        double currentVelocity = sim.getVelocityRadPerSec();
        sim.setState(0.0, currentVelocity);
    }

    @Override
    public void stop() {
        requestVoltage(Units.Volts.zero());
    }

}
