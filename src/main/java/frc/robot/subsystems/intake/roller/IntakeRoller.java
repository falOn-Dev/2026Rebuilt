package frc.robot.subsystems.intake.roller;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRoller extends SubsystemBase {
    private final IntakeRollerIO io;
    private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

    public IntakeRoller(IntakeRollerIO io) {
        super("IntakeRoller");
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake/Roller", inputs);
    }

    public Command genSpinRollerCommand(String name, DoubleSupplier speedSupplierRPS) {
        return this.runEnd(
            () -> io.requestVelocity(Units.RotationsPerSecond.of(speedSupplierRPS.getAsDouble())),
            io::stop
        ).withName("IntakeSpinRoller- ");
    }

    public Command getStopRollerCommand(String name) {
        return this.runOnce(() -> io.stop());
    }
}
