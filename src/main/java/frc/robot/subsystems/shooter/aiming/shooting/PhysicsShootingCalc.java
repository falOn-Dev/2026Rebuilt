package frc.robot.subsystems.shooter.aiming.shooting;

import java.awt.RadialGradientPaint;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutTime;
import frc.robot.subsystems.shooter.aiming.AimingConstants;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import tasks.physicsSim.PhysicsSimulation.ShooterConfig;

public class PhysicsShootingCalc implements ShootingCalc {

    private Translation3d aimPoint;

    private MutDistance effectiveDistance = Units.Meters.mutable(0.0);
    private MutAngle targetHoodAngle = Units.Rotations.mutable(0.0);
    private MutLinearVelocity targetFlywheelVelocity = Units.MetersPerSecond.mutable(0.0);
    private Rotation2d targetDriveHeading = Rotation2d.kZero;
    private MutTime tof = Units.Seconds.mutable(0.0);

    @Override
    public void calculate(Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation3d aimPoint) {
        // Get position of shooter
        Pose3d shooterPose = new Pose3d(robotPose).transformBy(HoodConstants.HOOD_BASE);
        Translation2d shooterPos = shooterPose.getTranslation().toTranslation2d();

        // Calculate vector from shooter to target
        Translation2d toTarget = aimPoint.toTranslation2d().minus(shooterPos);
        double distanceMeters = toTarget.getNorm();

        Translation2d radial = toTarget.div(toTarget.getNorm());
        Translation2d tangential = radial.rotateBy(Rotation2d.kCCW_90deg);

        Translation2d robotVelocity = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
        double radialVelocity = robotVelocity.getX() * radial.getX() + robotVelocity.getY() * radial.getY();
        double tangentialVelocity = robotVelocity.getX() * tangential.getX() + robotVelocity.getY() * tangential.getY();

        double hoodAngleRad = AimingConstants.hoodPolynomial.evaluate(distanceMeters, radialVelocity);

        double flywheelSpeedMps = AimingConstants.flywheelPolynomial.evaluate(distanceMeters, radialVelocity);

        double tofSeconds = AimingConstants.tofPolynomial.evaluate(distanceMeters, radialVelocity);

        Rotation2d baseHeading = new Rotation2d(toTarget.getX(), toTarget.getY());

        // Because we're moving sideways it's got some drift to it, calculate the amount
        // and compensate
        double sidewaysOffset = tangentialVelocity * tofSeconds;
        double leadAngleRad = Math.atan2(sidewaysOffset, distanceMeters);

        double correctedHeadingRad = baseHeading.getRadians() - leadAngleRad;
        Rotation2d correctedHeading = Rotation2d.fromRadians(correctedHeadingRad);

        this.aimPoint = aimPoint;
        this.effectiveDistance.mut_replace(distanceMeters, Units.Meters);
        this.targetHoodAngle.mut_replace(hoodAngleRad, Units.Radians);
        this.targetFlywheelVelocity.mut_replace(flywheelSpeedMps, Units.MetersPerSecond);
        this.tof.mut_replace(tofSeconds, Units.Seconds);
        this.targetDriveHeading = correctedHeading.rotateBy(Rotation2d.k180deg);

        Logger.recordOutput("Shooter/Aiming/effectiveDistance", this.effectiveDistance);
        Logger.recordOutput("Shooter/Aiming/targetHoodAngle", this.targetHoodAngle);
        Logger.recordOutput("Shooter/Aiming/targetFlywheelVelocity", this.targetFlywheelVelocity);
        Logger.recordOutput("Shooter/Aiming/targetDriveHeading", this.targetDriveHeading);
        Logger.recordOutput("Shooter/Aiming/tof", this.tof);
        Logger.recordOutput("Shooter/Aiming/aimPoint", this.aimPoint);

    }

    @Override
    public MutLinearVelocity getTargetFlywheelSurfaceVelocity() {
        return this.targetFlywheelVelocity;

    }

    @Override
    public MutAngle getTargetHoodAngle() {
        return this.targetHoodAngle;
    }

    @Override
    public Rotation2d getTargetDriveHeading() {
        return this.targetDriveHeading;
    }

    @Override
    public Translation3d getAimPoint() {
        return this.aimPoint;
    }

    @Override
    public MutTime getTOF() {
        return this.tof;
    }

}
