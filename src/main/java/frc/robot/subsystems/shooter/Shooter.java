package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.subsystems.shooter.aiming.AimingSystem;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.VirtualSubsystem;

public class Shooter extends VirtualSubsystem {
    public final Hood hood;
    public final Flywheel leftFlywheel;
    public final Flywheel rightFlywheel;
    public final AimingSystem aimingSystem;

    public Shooter(Hood hood, Flywheel leftFlywheel, Flywheel rightFlywheel, AimingSystem aimingSystem) {
        this.hood = hood;
        this.leftFlywheel = leftFlywheel;
        this.rightFlywheel = rightFlywheel;
        this.aimingSystem = aimingSystem;
    }

    public Command autoAimAtHubCommand(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds) {
        return Commands.parallel(
            aimingSystem.aimAtHub(pose, speeds, () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint)).repeatedly(),
            leftFlywheel.getDynamicRequestLinearVelocityCommand(aimingSystem.shootingCalc::getTargetFlywheelSurfaceVelocity),
            rightFlywheel.getDynamicRequestLinearVelocityCommand(aimingSystem.shootingCalc::getTargetFlywheelSurfaceVelocity),
            hood.getRequestDynamicAngleCommand(aimingSystem.shootingCalc::getTargetHoodAngle)
        );
    }

    @Override
    public void periodic() {
    }
}
