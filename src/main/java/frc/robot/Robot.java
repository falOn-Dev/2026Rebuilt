// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.deploy.IntakeDeploy;
import frc.robot.subsystems.intake.deploy.IntakeDeployConstants;
import frc.robot.subsystems.intake.deploy.IntakeDeployIO;
import frc.robot.subsystems.intake.deploy.IntakeDeployIOSim;
import frc.robot.subsystems.intake.deploy.IntakeDeployIOTalonFX;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.roller.IntakeRollerConstants;
import frc.robot.subsystems.intake.roller.IntakeRollerIO;
import frc.robot.subsystems.intake.roller.IntakeRollerIOSim;
import frc.robot.subsystems.intake.roller.IntakeRollerIOTalonFX;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.kicker.KickerConstants;
import frc.robot.subsystems.kicker.KickerIO;
import frc.robot.subsystems.kicker.KickerIOSim;
import frc.robot.subsystems.kicker.KickerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.aiming.AimingSystem;
import frc.robot.subsystems.shooter.aiming.shooting.InterpolatingShootingCalc;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodConstants;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFX;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferConstants;
import frc.robot.subsystems.transfer.TransferIO;
import frc.robot.subsystems.transfer.TransferIOSim;
import frc.robot.subsystems.transfer.TransferIOTalonFX;
import frc.robot.util.DoublePressTracker;
import frc.robot.util.FuelSim;
import frc.robot.util.LoggedTracer;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.VirtualSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    public final Drive drive;
    public final Intake intake;
    public final Kicker kicker;
    public final Transfer transfer;
    public final Shooter shooter;

    // Keep track of fuel in hopper, also from 6328
    public class SimFuelCount {
        public static final int capacity = 40;
        public static final double launchBPS = 5.0;

        public int fuelStored;

        public SimFuelCount(int fuelStored) {
            this.fuelStored = fuelStored;
        }
    }

    public FuelSim fuelSim;
    public SimFuelCount simFuelCount;

    public final CommandXboxController controller = new CommandXboxController(0);

    public Robot() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
                "GitDirty",
                switch (BuildConstants.DIRTY) {
                    case 0 -> "All changes committed";
                    case 1 -> "Uncommitted changes";
                    default -> "Unknown";
                });

        // Set up data receivers & replay source
        switch (Constants.CURRENT_MODE) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // Start AdvantageKit logger
        Logger.start();

        switch (Constants.CURRENT_MODE) {
            case REAL:
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));
                intake = new Intake(
                        new IntakeDeploy(
                                new IntakeDeployIOTalonFX(
                                        IntakeDeployConstants.LEFT_MOTOR_ID,
                                        IntakeDeployConstants.RIGHT_MOTOR_ID,
                                        IntakeDeployConstants.LEFT_INVERT,
                                        IntakeDeployConstants.RIGHT_INVERT,
                                        IntakeDeployConstants.GEARING,
                                        IntakeDeployConstants.PID_CONSTANTS,
                                        IntakeDeployConstants.FF_CONSTANTS,
                                        IntakeDeployConstants.MM_VELOCITY,
                                        IntakeDeployConstants.MM_ACCELERATION)),
                        new IntakeRoller(
                                new IntakeRollerIOTalonFX(
                                        IntakeRollerConstants.MOTOR_ID,
                                        IntakeRollerConstants.INVERTED,
                                        IntakeRollerConstants.GEARING,
                                        IntakeRollerConstants.PID_CONSTANTS,
                                        IntakeRollerConstants.FF_CONSTANTS)));

                kicker = new Kicker(
                        new KickerIOTalonFX(
                                KickerConstants.MOTOR_ID,
                                KickerConstants.INVERTED,
                                KickerConstants.GEARING,
                                KickerConstants.FF_CONSTANTS));

                transfer = new Transfer(
                        new TransferIOTalonFX(
                                TransferConstants.MOTOR_ID,
                                TransferConstants.INVERTED,
                                TransferConstants.GEARING,
                                TransferConstants.FF_CONSTANTS));

                shooter = new Shooter(
                        new Hood(new HoodIOTalonFX(
                                HoodConstants.MOTOR_ID,
                                HoodConstants.INVERTED,
                                HoodConstants.GEARING,
                                HoodConstants.PID_CONSTANTS,
                                HoodConstants.FF_CONSTANTS,
                                HoodConstants.MM_VELOCITY,
                                HoodConstants.MM_ACCEL)),
                        new Flywheel("LeftFlywheel", new FlywheelIOTalonFX(
                                FlywheelConstants.LEFT_MOTOR_ID,
                                FlywheelConstants.LEFT_INVERTED,
                                FlywheelConstants.GEARING,
                                FlywheelConstants.LEFT_PID,
                                FlywheelConstants.LEFT_FF)),
                        new Flywheel("RightFlywheel", new FlywheelIOTalonFX(
                                FlywheelConstants.RIGHT_MOTOR_ID,
                                FlywheelConstants.RIGHT_INVERTED,
                                FlywheelConstants.GEARING,
                                FlywheelConstants.RIGHT_PID,
                                FlywheelConstants.RIGHT_FF)),
                        new AimingSystem(new InterpolatingShootingCalc()));
                break;

            case SIM:
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));

                intake = new Intake(
                        new IntakeDeploy(
                                new IntakeDeployIOSim(
                                        IntakeDeployConstants.GEARING,
                                        IntakeDeployConstants.MOI)),
                        new IntakeRoller(
                                new IntakeRollerIOSim(
                                        IntakeRollerConstants.GEARING,
                                        IntakeRollerConstants.MOI)));

                kicker = new Kicker(
                        new KickerIOSim(
                                KickerConstants.GEARING,
                                KickerConstants.MOI));

                transfer = new Transfer(
                        new TransferIOSim(
                                TransferConstants.GEARING,
                                TransferConstants.ROLLER_MOI));

                shooter = new Shooter(
                        new Hood(new HoodIOSim(HoodConstants.GEARING, HoodConstants.PID_CONSTANTS,
                                HoodConstants.FF_CONSTANTS, HoodConstants.MOI)),
                        new Flywheel("LeftFlywheel",
                                new FlywheelIOSim(FlywheelConstants.GEARING, FlywheelConstants.FLYWHEEL_MOI)),
                        new Flywheel("RightFlywheel",
                                new FlywheelIOSim(FlywheelConstants.GEARING, FlywheelConstants.FLYWHEEL_MOI)),
                        new AimingSystem(new InterpolatingShootingCalc()));

                fuelSim = new FuelSim();
                simFuelCount = new SimFuelCount(8);
                configureFuelSim();

                break;
            default:
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        });

                intake = new Intake(
                        new IntakeDeploy(new IntakeDeployIO() {
                        }),
                        new IntakeRoller(new IntakeRollerIO() {
                        }));

                kicker = new Kicker(
                        new KickerIO() {
                        });

                transfer = new Transfer(
                        new TransferIO() {
                        });

                shooter = new Shooter(
                        new Hood(new HoodIO() {
                        }),
                        new Flywheel("LeftFlywheel", new FlywheelIO() {
                        }),
                        new Flywheel("RightFlywheel", new FlywheelIO() {
                        }),
                        new AimingSystem(new InterpolatingShootingCalc()));
                break;
        }

        configureBindings();
    }

    public void configureBindings() {
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRawAxis(3)));

        controller.axisGreaterThan(5, 0.5).whileTrue(
                Commands.parallel(
                        shooter.autoAimAtHubCommand(drive::getPose, drive::getChassisSpeeds),
                        DriveCommands.joystickDriveAtAngle(
                                drive,
                                () -> -controller.getLeftY(),
                                () -> -controller.getLeftX(),
                                () -> shooter.aimingSystem.shootingCalc.getTargetDriveHeading())));

        DoublePressTracker intakeRetractTracker = new DoublePressTracker(controller.leftTrigger());
        Trigger retractIntake = new Trigger(() -> intakeRetractTracker.get());

        // Deploy intake while left trigger is held and the intake has been homed since
        // robot startup
        // If the intake is already deployed it should just spin the roller up, untested
        // currently
        controller.leftTrigger().and(intake.isInit.negate()).whileTrue(intake.intake());

        // Pull intake in if its deployed and double tap trigger
        retractIntake.and(intake.isDeployed).onTrue(intake.stow());

        controller.x().whileTrue(
                Commands.parallel(
                        kicker.feed(),
                        transfer.feed()));
    }

    public void configureFuelSim() {
        fuelSim.registerRobot(
                DriveConstants.ROBOT_WIDTH,
                DriveConstants.ROBOT_LENGTH,
                DriveConstants.BUMPER_HEIGHT,
                drive::getPose,
                drive::getChassisSpeeds);

        fuelSim.registerIntake(
                DriveConstants.ROBOT_LENGTH.div(2.0),
                DriveConstants.ROBOT_LENGTH.div(2.0).plus(Units.Inches.of(12.0)),
                DriveConstants.ROBOT_WIDTH.div(-2.0),
                DriveConstants.ROBOT_WIDTH.div(2.0),
            () -> 
                intake.isDeployed.getAsBoolean() 
            && intake.roller.inputs.angularVelocity.gt(Units.RPM.of(100.0))
            && simFuelCount.fuelStored < SimFuelCount.capacity,
            () -> simFuelCount.fuelStored = Math.min(simFuelCount.fuelStored + 1, SimFuelCount.capacity)
        );

        fuelSim.setSubticks(1);
        fuelSim.start();
        // fuelSim.spawnStartingFuel();

        RobotModeTriggers.autonomous()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    fuelSim.clearFuel();
                                    // fuelSim.spawnStartingFuel();
                                    simFuelCount.fuelStored = 40;
                                }));
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        LoggedTracer.reset();
        PhoenixUtil.refreshAll();
        LoggedTracer.record("PhoenixRefresh");

        CommandScheduler.getInstance().run();
        LoggedTracer.record("CommandRun");

        VirtualSubsystem.periodicAll();
        LoggedTracer.record("VirtualSubsystemRun");
    }

    @Override
    public void simulationPeriodic() {
        if (fuelSim != null) {
            fuelSim.updateSim();
            Logger.recordOutput("FuelSim/FuelStored", simFuelCount.fuelStored);

            if(kicker.inputs.angularVelocity.gt(Units.RPM.of(500)) && simFuelCount.fuelStored > 0) {
                simFuelCount.fuelStored--;
                fuelSim.launchFuel(
                    shooter.leftFlywheel.inputs.linearVelocity,
                    Units.Degrees.of(90.0).minus(shooter.hood.inputs.angularPosition),
                    Units.Degrees.zero(),
                    Units.Inches.of(19.0)
                );
            }
        }
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
        CommandScheduler.getInstance().schedule(intake.home());
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}
