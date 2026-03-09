package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intake.deploy.IntakeDeploy;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.util.VirtualSubsystem;

public class Intake extends VirtualSubsystem {
    public final IntakeDeploy deploy;
    public final IntakeRoller roller;

    public enum IntakeState {
        INIT,
        HOMED,
        STOWED,
        DEPLOYED
    }

    private IntakeState state = IntakeState.INIT;

    public Trigger isInit = new Trigger(() -> state == IntakeState.INIT);
    public Trigger isHomed = new Trigger(() -> state == IntakeState.HOMED);
    public Trigger isStowed = new Trigger(() -> state == IntakeState.STOWED);
    public Trigger isDeployed = new Trigger(() -> state == IntakeState.DEPLOYED);

    public Intake(IntakeDeploy deploy, IntakeRoller roller) {
        this.deploy = deploy;
        this.roller = roller;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Intake/State", this.state.name());
    }

    public Command home() {
        return Commands.sequence(
                this.deploy.getHomeCommand().withTimeout(1.0),
                Commands.runOnce(() -> this.state = IntakeState.HOMED),
                this.stow());
    }

    public Command intake() {
        return Commands.parallel(
                this.roller.getIntakeCommand(),
                Commands.sequence(
                        this.deploy.getExtendCommand(),
                        Commands.runOnce(() -> this.state = IntakeState.DEPLOYED)));
    }

    public Command stow() {
        return Commands.parallel(
                this.roller.getStopRollerCommand(),
                Commands.sequence(
                        this.deploy.getStowCommand(),
                        Commands.runOnce(() -> this.state = IntakeState.STOWED)));
    }
}
