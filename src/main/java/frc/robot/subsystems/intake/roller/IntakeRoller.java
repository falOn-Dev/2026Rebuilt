package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRoller extends SubsystemBase {
    private final IntakeRollerIO io;
    public final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

    public IntakeRoller(IntakeRollerIO io) {
        super("IntakeRoller");
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Roller", inputs);
    }

    public Command getIntakeCommand() {
        return this.runEnd(
            () -> io.requestVelocity(IntakeRollerConstants.INTAKE_VELOCITY),
            io::stop
        ).withName("IntakeSpinRoller");
    }

    public Command getStopRollerCommand() {
        return this.runOnce(() -> io.stop());
    }
}
