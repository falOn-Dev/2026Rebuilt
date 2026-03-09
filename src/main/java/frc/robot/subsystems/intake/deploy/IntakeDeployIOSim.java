package frc.robot.subsystems.intake.deploy;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.FFConstants;
import frc.robot.util.PIDConstants;
import frc.robot.util.UnitConversions;

public class IntakeDeployIOSim implements IntakeDeployIO {
    private final DCMotorSim sim;
    private final PIDController pid;

    private boolean isOpenLoop = true;
    private MutVoltage cachedVoltage = Units.Volts.mutable(0.0);

    public IntakeDeployIOSim(
            double gearing,
            MomentOfInertia moi) {
        sim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60Foc(2),
                        moi.in(Units.KilogramSquareMeters),
                        gearing),
                DCMotor.getKrakenX60Foc(2));

        pid = new PIDController(100.0, 0.0, 0.0);
    }

    @Override
    public void updateInputs(IntakeDeployInputs inputs) {

        if (isOpenLoop) {
            pid.reset();
            pid.setSetpoint(0.0);
            sim.setInputVoltage(cachedVoltage.in(Units.Volts));
        } else {
            cachedVoltage.mut_replace(pid.calculate(
                    UnitConversions.angularPositionToLinearPosition(sim.getAngularPosition(),
                            IntakeDeployConstants.PINION_PITCH_RADIUS).in(Units.Meters)),
                    Units.Volts);
            sim.setInputVoltage(cachedVoltage.in(Units.Volts));
        }

        sim.update(Constants.LOOP_TIME);

        inputs.isConnected = true;

        inputs.leftAppliedVoltage.mut_replace(cachedVoltage);
        inputs.leftAngularPosition.mut_replace(sim.getAngularPosition());
        inputs.leftLinearPosition.mut_replace(
                UnitConversions.angularPositionToLinearPosition(sim.getAngularPosition(),
                        IntakeDeployConstants.PINION_PITCH_RADIUS));

        inputs.leftAngularVelocity.mut_replace(sim.getAngularVelocity());
        inputs.leftLinearVelocity.mut_replace(
                UnitConversions.angularVelocityToLinearVelocity(
                        sim.getAngularVelocity(), IntakeDeployConstants.PINION_PITCH_RADIUS));

        inputs.leftSupplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Units.Amps);
        inputs.leftStatorCurrent.mut_replace(inputs.leftSupplyCurrent);

        inputs.rightAppliedVoltage.mut_replace(cachedVoltage);
        inputs.rightAngularPosition.mut_replace(sim.getAngularPosition());
        inputs.rightLinearPosition.mut_replace(
                UnitConversions.angularPositionToLinearPosition(sim.getAngularPosition(),
                        IntakeDeployConstants.PINION_PITCH_RADIUS));

        inputs.rightAngularVelocity.mut_replace(sim.getAngularVelocity());
        inputs.rightLinearVelocity.mut_replace(
                UnitConversions.angularVelocityToLinearVelocity(
                        sim.getAngularVelocity(), IntakeDeployConstants.PINION_PITCH_RADIUS));

        inputs.rightSupplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Units.Amps);
        inputs.rightStatorCurrent.mut_replace(inputs.leftSupplyCurrent);
    }

    @Override
    public void requestLeftVoltage(Voltage voltage) {
        isOpenLoop = true;
        cachedVoltage.mut_replace(voltage);
    }

    @Override
    public void requestRightVoltage(Voltage voltage) {
        isOpenLoop = true;
        cachedVoltage.mut_replace(voltage);
    }

    @Override
    public void requestExtension(Distance extension) {
        isOpenLoop = false;
        pid.setSetpoint(extension.in(Units.Meters));
    }

    @Override
    public void stopLeft() {
        requestLeftVoltage(Units.Volts.zero());
    }

    @Override
    public void stopRight() {
        requestRightVoltage(Units.Volts.zero());
    }

    @Override
    public void stop() {
        stopLeft();
        stopRight();
    }

    @Override
    public void setLeftEncoderPosition(Angle position) {
        double currentVelocity = sim.getAngularVelocityRadPerSec();
        sim.setState(position.in(Units.Radians), currentVelocity);
    }

    @Override
    public void resetLeftEncoderPosition() {
        double currentVelocity = sim.getAngularVelocityRadPerSec();
        sim.setState(0.0, currentVelocity);
    }

    @Override
    public void setRightEncoderPosition(Angle position) {
        double currentVelocity = sim.getAngularVelocityRadPerSec();
        sim.setState(position.in(Units.Radians), currentVelocity);
    }

    @Override
    public void resetRightEncoderPosition() {
        double currentVelocity = sim.getAngularVelocityRadPerSec();
        sim.setState(0.0, currentVelocity);
    }
}
