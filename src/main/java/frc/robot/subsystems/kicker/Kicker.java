package frc.robot.subsystems.kicker;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kicker extends SubsystemBase {
    private final KickerIO io;
    private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

    public Kicker(KickerIO io) {
        super("Kicker");
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Kicker", inputs);
    }

    public Command feed() {
        return this.runEnd(
            () -> io.requestVelocity(KickerConstants.FEED_SPEED),
            io::stop
        ).withName("KickerFeed");
    }
}
