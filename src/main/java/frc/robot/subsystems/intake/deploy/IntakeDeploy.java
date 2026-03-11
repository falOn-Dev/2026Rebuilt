package frc.robot.subsystems.intake.deploy;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeDeploy extends SubsystemBase {
    private final IntakeDeployIO io;
    public final IntakeDeployInputsAutoLogged inputs = new IntakeDeployInputsAutoLogged();

    public boolean isLeftHomed = false;
    public Debouncer leftHomedDebouncer = new Debouncer(0.1);

    public boolean isRightHomed = false;
    public Debouncer rightHomedDebouncer = new Debouncer(0.1);


    public IntakeDeploy(IntakeDeployIO io) {
        super("IntakeDeploy");
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Deploy", inputs);


        Logger.recordOutput("Intake/Deploy/isLeftHomed", isLeftHomed);
        Logger.recordOutput("Intake/Deploy/isRightHomed", isRightHomed);
    }

    public Command getHomeCommand() {
        return this.runEnd(
            () -> {
                if(leftHomedDebouncer.calculate(inputs.leftStatorCurrent.gte(IntakeDeployConstants.HOMING_THRESHOLD))) {
                    isLeftHomed = true;
                }

                if(rightHomedDebouncer.calculate(inputs.rightStatorCurrent.gte(IntakeDeployConstants.HOMING_THRESHOLD))){
                    isRightHomed = true;
                }

                if(!isLeftHomed) { io.requestLeftVoltage(IntakeDeployConstants.HOMING_VOLTAGE); }
                if(!isRightHomed) { io.requestRightVoltage(IntakeDeployConstants.HOMING_VOLTAGE); }
            },
            () -> {
                io.resetLeftEncoderPosition();
                io.resetRightEncoderPosition();
                isLeftHomed = false;
                isRightHomed = false;
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
