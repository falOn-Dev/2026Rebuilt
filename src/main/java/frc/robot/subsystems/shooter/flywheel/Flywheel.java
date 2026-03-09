package frc.robot.subsystems.shooter.flywheel;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
    private final FlywheelIO io;
    public final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    public Flywheel(String name, FlywheelIO io) {
        super(name);
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/" + this.getName(), inputs);
        this.setDefaultCommand(this.idle());
    }

    public Command getDynamicRequestAngularVelocityCommand(Supplier<AngularVelocity> velocitySupplier) {
        return this.run(() -> io.requestAngularVelocity(velocitySupplier.get()));
    }

    public Command getDynamicRequestLinearVelocityCommand(Supplier<LinearVelocity> velocitySupplier) {
        return this.run(() -> io.requestLinearVelocity(velocitySupplier.get()));
    }

    public Command idle() {
        return this.run(() -> io.requestAngularVelocity(FlywheelConstants.IDLE_VELOCITY));
    }

}
