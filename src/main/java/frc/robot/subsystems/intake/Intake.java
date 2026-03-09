package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intake.deploy.IntakeDeploy;
import frc.robot.subsystems.intake.roller.IntakeRoller;

public class Intake {
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

    public Command home() {
        return Commands.sequence(
            this.deploy.getHomeCommand(),
            Commands.runOnce(() -> this.state = IntakeState.HOMED),
            this.stow()
        );
    }

    public Command intake() {
        return Commands.parallel(
            this.deploy.getExtendCommand(),
            Commands.sequence(
                this.roller.getIntakeCommand(),
                Commands.runOnce(() -> this.state = IntakeState.DEPLOYED)
            )
        );
    }

    public Command stow() {
        return Commands.parallel(
            this.deploy.getStowCommand(),
            Commands.sequence(
                this.roller.getStopRollerCommand(),
                Commands.runOnce(() -> this.state = IntakeState.STOWED)
            )
        );
    }
}
