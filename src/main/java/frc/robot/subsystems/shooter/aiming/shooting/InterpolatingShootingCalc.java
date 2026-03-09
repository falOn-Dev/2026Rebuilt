package frc.robot.subsystems.shooter.aiming.shooting;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.shooter.aiming.AimingConstants;

public class InterpolatingShootingCalc implements ShootingCalc {
    private static final Time lookaheadTime = Units.Seconds.of(0.2);

    private Translation3d aimPoint;

    private MutDistance effectiveDistance = Units.Meters.mutable(0.0);
    private MutAngle targetHoodAngle = Units.Rotations.mutable(0.0);
    private MutLinearVelocity targetFlywheelVelocity = Units.MetersPerSecond.mutable(0.0);
    private Rotation2d targetDriveHeading = Rotation2d.kZero;
    private MutTime tof = Units.Seconds.mutable(0.0);

    @Override
    public void calculate(Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation3d aimPoint) {
        Translation2d robotPos = robotPose.getTranslation();
        this.aimPoint = aimPoint;

        double predictedX = robotPos.getX() + fieldSpeeds.vxMetersPerSecond * lookaheadTime.in(Units.Seconds);
        double predictedY = robotPos.getY() + fieldSpeeds.vyMetersPerSecond * lookaheadTime.in(Units.Seconds);

        double predictedToTargetX = this.aimPoint.getX() - predictedX;
        double predictedToTargetY = this.aimPoint.getY() - predictedY;

        // Calculate distance and heading from predicted position to target
        this.targetDriveHeading = Rotation2d.fromRadians(Math.atan2(predictedToTargetY, predictedToTargetX));
        this.effectiveDistance.mut_replace(Math.hypot(predictedToTargetX, predictedToTargetY), Units.Meters);
        Logger.recordOutput("Shooter/Aiming/effectiveDistance", this.effectiveDistance);

        // Interpolate Shot Data
        ShotData interpolatedShot = AimingConstants.SHOT_DATA_MAP.get(this.effectiveDistance.in(Units.Meters));

        this.targetHoodAngle.mut_replace(interpolatedShot.targetHoodAngle());
        this.targetFlywheelVelocity.mut_replace(interpolatedShot.targetFlywheelVelocity());
        tof.mut_replace(AimingConstants.TOF_MAP.get(this.effectiveDistance.in(Units.Meters)), Units.Seconds);

        Logger.recordOutput("Shooter/Aiming/aimPoint", this.aimPoint);
        Logger.recordOutput("Shooter/Aiming/predictedPose", new Pose2d(
                new Translation2d(
                        predictedX,
                        predictedY),
                this.targetDriveHeading));

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
    public Rotation2d getTargetDriveHeading(){
        return this.targetDriveHeading;
    }

    @Override
    public Translation3d getAimPoint() {
        return this.aimPoint;
    }

    @Override
    public MutTime getTOF() {
        return this.tof; }
}
