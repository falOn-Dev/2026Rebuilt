package frc.robot.subsystems.shooter.aiming;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.aiming.shooting.ShootingCalc;

public class AimingSystem extends SubsystemBase {
    public final ShootingCalc shootingCalc;

    public AimingSystem(ShootingCalc shootingCalc) {
        super("Aiming");
        this.shootingCalc = shootingCalc;
    }

    public Command aimAtHub(Supplier<Pose2d> pose, Supplier<ChassisSpeeds> speeds, Supplier<Translation3d> aimPoint) {
        return this.runOnce(() -> this.shootingCalc.calculate(pose.get(), speeds.get(), aimPoint.get()))
                .withName("AimAtHub").ignoringDisable(true);
    }
}
