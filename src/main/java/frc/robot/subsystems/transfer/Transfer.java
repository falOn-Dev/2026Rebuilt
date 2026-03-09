package frc.robot.subsystems.transfer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Transfer extends SubsystemBase {
    private final TransferIO io;
    private final TransferIOInputsAutoLogged inputs = new TransferIOInputsAutoLogged();

    public Transfer(TransferIO io) {
        super("Transfer");
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Transfer", inputs);
    }

    public Command feed() {
        return this.runEnd(
            () -> io.requestVelocity(TransferConstants.FEED_VELOCITY),
            io::stop
        ).withName("TransferFeed");
    }
}
