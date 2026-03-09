package frc.robot.subsystems.shooter.hood;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
    private HoodIO io;
    private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public Hood(HoodIO io){
        super("Hood");
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);
    }

    public Command home() {
        return Commands.sequence(
            this.run(() -> io.requestVoltage(HoodConstants.HOMING_VOLTAGE)).until(() -> inputs.statorCurrent.gte(HoodConstants.HOMING_THRESHOLD)),
            this.runOnce(() -> io.resetEncoderPosition())
        );
    }

    public Command getRequestDynamicAngleCommand(Supplier<Angle> angleSupplier) {
        return this.run(() -> io.requestPosition(angleSupplier.get()));
    }
}
