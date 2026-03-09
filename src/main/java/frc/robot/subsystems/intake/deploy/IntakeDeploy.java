package frc.robot.subsystems.intake.deploy;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeDeploy extends SubsystemBase {
    private final IntakeDeployIO io;
    private final IntakeDeployInputsAutoLogged inputs = new IntakeDeployInputsAutoLogged();

    private final LinearFilter leftCurrentSpikeFilter = LinearFilter.singlePoleIIR(0.05, Constants.LOOP_TIME);
    private final Debouncer leftCurrentSpikeDebouncer = new Debouncer(0.1, DebounceType.kRising);
    public boolean isLeftHomed = false;

    private final LinearFilter rightCurrentSpikeFilter = LinearFilter.singlePoleIIR(0.05, Constants.LOOP_TIME);
    public boolean isRightHomed = false;
    private final Debouncer rightCurrentSpikeDebouncer = new Debouncer(0.1, DebounceType.kRising);

    public IntakeDeploy(IntakeDeployIO io) {
        super("IntakeDeploy");
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Deploy", inputs);

        leftCurrentSpikeFilter.calculate(inputs.leftStatorCurrent.in(Units.Amps));
        rightCurrentSpikeFilter.calculate(inputs.rightStatorCurrent.in(Units.Amps));

        isLeftHomed = leftCurrentSpikeDebouncer.calculate(leftCurrentSpikeFilter.lastValue() > IntakeDeployConstants.HOMING_THRESHOLD.in(Units.Amps));
        isLeftHomed = rightCurrentSpikeDebouncer.calculate(rightCurrentSpikeFilter.lastValue() > IntakeDeployConstants.HOMING_THRESHOLD.in(Units.Amps));


        Logger.recordOutput("Intake/Deploy/isLeftHomed", isLeftHomed);
        Logger.recordOutput("Intake/Deploy/isRightHomed", isRightHomed);
    }

    public Command getHomeCommand() {
        return this.runEnd(
            () -> {
                if(!isLeftHomed) { io.requestLeftVoltage(IntakeDeployConstants.HOMING_VOLTAGE); }
                if(!isRightHomed) { io.requestRightVoltage(IntakeDeployConstants.HOMING_VOLTAGE); }
            },
            () -> {
                io.resetLeftEncoderPosition();
                io.resetRightEncoderPosition();
            }
        ).until(() -> isLeftHomed && isRightHomed).andThen(getStowCommand());
    }

	public Command getStowCommand() {
        return this.runOnce(() -> io.requestExtension(IntakeDeployConstants.STOW_POSITION));
	}

    public Command getExtendCommand() {
        return this.runOnce(() -> io.requestExtension(IntakeDeployConstants.INTAKE_POSITION));
    }


}
