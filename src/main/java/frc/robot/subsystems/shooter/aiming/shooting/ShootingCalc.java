package frc.robot.subsystems.shooter.aiming.shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutTime;

public interface ShootingCalc {
	public void calculate(Pose2d robotPose, ChassisSpeeds fieldSpeeds, Translation3d aimPoint);

	public MutLinearVelocity getTargetFlywheelSurfaceVelocity();
	public MutAngle getTargetHoodAngle();
	public Rotation2d getTargetDriveHeading();

	public Translation3d getAimPoint();

	public MutTime getTOF();
}
