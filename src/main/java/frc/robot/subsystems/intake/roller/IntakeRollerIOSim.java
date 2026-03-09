package frc.robot.subsystems.intake.roller;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.FFConstants;
import frc.robot.util.PIDConstants;

public class IntakeRollerIOSim implements IntakeRollerIO {
    private final DCMotorSim sim;

    private final PIDController pid;
    private final SimpleMotorFeedforward feedforward;

    private final MutVoltage cachedVoltage = Units.Volts.mutable(0.0);
    private boolean isOpenLoop = true;

    public IntakeRollerIOSim(
            double gearing,
            PIDConstants pidConstants,
            FFConstants ffConstants,
            MomentOfInertia moi) {
        sim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX44Foc(1),
                        moi.in(Units.KilogramSquareMeters),
                        gearing),
                DCMotor.getKrakenX44Foc(1));

        pid = new PIDController(pidConstants.kP(), pidConstants.kI(), pidConstants.kD());
        feedforward = new SimpleMotorFeedforward(ffConstants.kS(), ffConstants.kV(), ffConstants.kA());
    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        if (isOpenLoop) {
            pid.reset();
            pid.setSetpoint(0.0);
            sim.setInputVoltage(cachedVoltage.in(Units.Volts));
        } else {
            double output = pid.calculate(sim.getAngularVelocity().in(Units.RotationsPerSecond));
            output += feedforward.calculate(pid.getSetpoint());
            cachedVoltage.mut_replace(output, Units.Volts);
            sim.setInputVoltage(cachedVoltage.in(Units.Volts));
        }

        sim.update(Constants.LOOP_TIME);

        inputs.isConnected = true;
        inputs.appliedVoltage.mut_replace(cachedVoltage);
        inputs.angularPosition.mut_replace(sim.getAngularPosition());
        inputs.angularVelocity.mut_replace(sim.getAngularVelocity());
        inputs.linearVelocity.mut_replace(
                inputs.angularVelocity.in(Units.RadiansPerSecond) * IntakeRollerConstants.ROLLER_RADIUS.in(Units.Meters),
                Units.MetersPerSecond);
        inputs.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Units.Amps);
        inputs.statorCurrent.mut_replace(inputs.supplyCurrent);
    }

    @Override
    public void requestVoltage(Voltage volts) {
        isOpenLoop = true;
        cachedVoltage.mut_replace(volts);
    }

    @Override
    public void requestVelocity(AngularVelocity velocity) {
        isOpenLoop = false;
        pid.setSetpoint(velocity.in(Units.RotationsPerSecond));
    }

    @Override
    public void stop() {
        requestVoltage(Units.Volts.zero());
    }
}
