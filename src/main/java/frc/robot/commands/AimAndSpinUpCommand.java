package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.commands.CompleteShootSequence;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hooper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.FuelSim;

import java.util.function.DoubleSupplier;

public class AimAndSpinUpCommand extends Command {

    private static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.552, 4.021);
    private static final Translation2d RED_HUB_CENTER  = new Translation2d(11.961, 4.021);

    private static final String LIMELIGHT_NAME              = "limelight";
    private static final double MAX_VISION_TRUST_DISTANCE_M = 5.0;
    private static final int    MIN_TAG_COUNT               = 1;
    private static final double VISION_BASE_STD_XY         = 0.3;
    private static final double VISION_BASE_STD_THETA      = 9999; // never let LL override gyro heading

    private static final double AIM_MAX_ANGULAR_RATE_RADPS =
            edu.wpi.first.math.util.Units.rotationsToRadians(2.0);

    private static final double KP_ROTATION       = 0.08;
    private static final double MIN_COMMAND        = 0.03;
    private static final double ERROR_DEADZONE_DEG = 1.5;
    private static final double ALIGNED_TOLERANCE_DEG = 2.5;
    private static final int    TOLERANCE_LOOP_COUNT  = 5;

    private static final double SHOOTER_MIN_DISTANCE_M = 1.0;
    private static final double SHOOTER_MAX_DISTANCE_M = 6.0;

    private static final double JOYSTICK_DEADBAND = 0.1;

    private final CommandSwerveDrivetrain drivetrain;
    private final ShooterSubsystem shooter;
    private final HopperSubsystem hopper;
    private final FeederSubsystem feeder;
    private final FuelSim fuelSim;
    private final DoubleSupplier leftYSupplier;
    private final DoubleSupplier leftXSupplier;
    private final double maxSpeed;
    private final double maxAngularRate;
    private final boolean finishWhenAligned;

    private int withinToleranceCount = 0;
    private boolean hasFired = false;

    public AimAndSpinUpCommand(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            FeederSubsystem feeder,
            FuelSim fuelSim,
            DoubleSupplier leftYSupplier,
            DoubleSupplier leftXSupplier,
            double maxSpeed,
            double maxAngularRate) {
        this(drivetrain, shooter, hopper, feeder, fuelSim, leftYSupplier, leftXSupplier, maxSpeed, maxAngularRate, false);
    }

    public AimAndSpinUpCommand(
            CommandSwerveDrivetrain drivetrain,
            ShooterSubsystem shooter,
            HopperSubsystem hopper,
            FeederSubsystem feeder,
            FuelSim fuelSim,
            DoubleSupplier leftYSupplier,
            DoubleSupplier leftXSupplier,
            double maxSpeed,
            double maxAngularRate,
            boolean finishWhenAligned) {
        this.drivetrain        = drivetrain;
        this.shooter           = shooter;
        this.hopper            = hopper;
        this.feeder            = feeder;
        this.fuelSim           = fuelSim;
        this.leftYSupplier     = leftYSupplier;
        this.leftXSupplier     = leftXSupplier;
        this.maxSpeed          = maxSpeed;
        this.maxAngularRate    = maxAngularRate;
        this.finishWhenAligned = finishWhenAligned;

        addRequirements(drivetrain, shooter);
    }

    @Override
    public void initialize() {
        withinToleranceCount = 0;
        hasFired = false;
        SmartDashboard.putBoolean("AimAndSpinUp/Running",      true);
        SmartDashboard.putBoolean("AimAndSpinUp/ReadyToShoot", false);
    }

    @Override
    public void execute() {

        updateVisionPoseEstimate();

        double rawY = leftYSupplier.getAsDouble();
        double rawX = leftXSupplier.getAsDouble();
        double forwardSpeed = (Math.abs(rawY) > JOYSTICK_DEADBAND ? -rawY : 0.0) * maxSpeed;
        double strafeSpeed  = (Math.abs(rawX) > JOYSTICK_DEADBAND ? -rawX : 0.0) * maxSpeed;

        Pose2d robotPose = drivetrain.getState().Pose;

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        Translation2d hubCenter = (alliance == Alliance.Blue) ? BLUE_HUB_CENTER : RED_HUB_CENTER;

        Translation2d toHub = hubCenter.minus(robotPose.getTranslation());
        double distanceToHub = toHub.getNorm();
        Rotation2d targetHeading = new Rotation2d(toHub.getX(), toHub.getY());

        double rotationErrorDeg = getModuloRotation(
                targetHeading.getDegrees() - robotPose.getRotation().getDegrees()
        );

        double rotationOutput;
        if (Math.abs(rotationErrorDeg) > ERROR_DEADZONE_DEG) {
            rotationOutput = KP_ROTATION * rotationErrorDeg;
            rotationOutput += (rotationErrorDeg > 0) ? MIN_COMMAND : -MIN_COMMAND;
        } else {
            rotationOutput = KP_ROTATION * rotationErrorDeg;
        }
        rotationOutput = Math.max(-AIM_MAX_ANGULAR_RATE_RADPS, Math.min(AIM_MAX_ANGULAR_RATE_RADPS, rotationOutput));

        boolean rotationAligned = Math.abs(rotationErrorDeg) <= ALIGNED_TOLERANCE_DEG;
        if (rotationAligned) withinToleranceCount++;
        else                 withinToleranceCount = 0;

        boolean distanceInRange = distanceToHub >= SHOOTER_MIN_DISTANCE_M
                               && distanceToHub <= SHOOTER_MAX_DISTANCE_M;
        if (distanceInRange) {
            shooter.setVelocityForDistance(distanceToHub);
        } else {
            shooter.stop();
        }

        boolean shooterReady = shooter.atTargetVelocity();
        boolean readyToShoot = rotationAligned && shooterReady && distanceInRange;
        if (readyToShoot && !hasFired) {
            hasFired = true;
            new CompleteShootSequence(shooter, hopper, feeder, distanceToHub, fuelSim).schedule();
        }

        drivetrain.setControl(
            new SwerveRequest.FieldCentric()
                .withVelocityX(forwardSpeed)
                .withVelocityY(strafeSpeed)
                .withRotationalRate(rotationOutput)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        );

        SmartDashboard.putString( "AimAndSpinUp/Alliance",              alliance.toString());
        SmartDashboard.putNumber( "AimAndSpinUp/Robot_X",               robotPose.getX());
        SmartDashboard.putNumber( "AimAndSpinUp/Robot_Y",               robotPose.getY());
        SmartDashboard.putNumber( "AimAndSpinUp/Robot_Heading_deg",     robotPose.getRotation().getDegrees());
        SmartDashboard.putNumber( "AimAndSpinUp/Target_Heading_deg",    targetHeading.getDegrees());
        SmartDashboard.putNumber( "AimAndSpinUp/Rotation_Error_deg",    rotationErrorDeg);
        SmartDashboard.putNumber( "AimAndSpinUp/Rotation_Output_radps", rotationOutput);
        SmartDashboard.putBoolean("AimAndSpinUp/Rotation_Aligned",      rotationAligned);
        SmartDashboard.putNumber( "AimAndSpinUp/Tolerance_Count",       withinToleranceCount);
        SmartDashboard.putNumber( "AimAndSpinUp/Distance_To_Hub_m",     distanceToHub);
        SmartDashboard.putBoolean("AimAndSpinUp/Distance_In_Range",     distanceInRange);
        SmartDashboard.putBoolean("AimAndSpinUp/Shooter_At_Target",     shooterReady);
        SmartDashboard.putString( "AimAndSpinUp/Active_Profile",        shooter.getActiveProfileName());
        SmartDashboard.putBoolean("AimAndSpinUp/ReadyToShoot",          readyToShoot);
        SmartDashboard.putNumber( "AimAndSpinUp/Forward_Speed_mps",     forwardSpeed);
        SmartDashboard.putNumber( "AimAndSpinUp/Strafe_Speed_mps",      strafeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        SmartDashboard.putBoolean("AimAndSpinUp/Running",      false);
        SmartDashboard.putBoolean("AimAndSpinUp/ReadyToShoot", false);
    }

    @Override
    public boolean isFinished() {
        return finishWhenAligned && withinToleranceCount >= TOLERANCE_LOOP_COUNT;
    }


    private void updateVisionPoseEstimate() {
        LimelightHelpers.SetRobotOrientation(
                LIMELIGHT_NAME,
                drivetrain.getState().Pose.getRotation().getDegrees(),
                0.0, 0.0, 0.0, 0.0, 0.0
        );

        LimelightHelpers.PoseEstimate estimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);

        if (estimate == null) return;
        if (estimate.tagCount < MIN_TAG_COUNT) return;
        if (estimate.avgTagDist > MAX_VISION_TRUST_DISTANCE_M) return;
        if (estimate.latency > 0.5) return; // frame is too old

        double px = estimate.pose.getX();
        double py = estimate.pose.getY();
        if (px < -0.5 || px > 17.0 || py < -0.5 || py > 8.5) return; // off-field sanity check

        double xyStdDev = VISION_BASE_STD_XY * Math.max(1.0, estimate.avgTagDist);

        drivetrain.addVisionMeasurement(
                estimate.pose,
                estimate.timestampSeconds,
                VecBuilder.fill(xyStdDev, xyStdDev, VISION_BASE_STD_THETA)
        );

        SmartDashboard.putBoolean("Vision/MeasurementAccepted", true);
        SmartDashboard.putNumber( "Vision/TagCount",             estimate.tagCount);
        SmartDashboard.putNumber( "Vision/AvgTagDist_m",         estimate.avgTagDist);
        SmartDashboard.putNumber( "Vision/Latency_s",            estimate.latency);
        SmartDashboard.putNumber( "Vision/Pose_X",               px);
        SmartDashboard.putNumber( "Vision/Pose_Y",               py);
        SmartDashboard.putNumber( "Vision/XY_StdDev",            xyStdDev);
    }

    public boolean isReadyToShoot() {
        Pose2d robotPose = drivetrain.getState().Pose;
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        Translation2d hubCenter = (alliance == Alliance.Blue) ? BLUE_HUB_CENTER : RED_HUB_CENTER;
        Translation2d toHub = hubCenter.minus(robotPose.getTranslation());
        double distanceToHub = toHub.getNorm();
        Rotation2d targetHeading = new Rotation2d(toHub.getX(), toHub.getY());

        double errorDeg = getModuloRotation(
                targetHeading.getDegrees() - robotPose.getRotation().getDegrees()
        );

        return Math.abs(errorDeg) <= ALIGNED_TOLERANCE_DEG
            && shooter.atTargetVelocity()
            && distanceToHub >= SHOOTER_MIN_DISTANCE_M
            && distanceToHub <= SHOOTER_MAX_DISTANCE_M;
    }

    public double getDistanceToHub() {
        Pose2d robotPose = drivetrain.getState().Pose;
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        Translation2d hubCenter = (alliance == Alliance.Blue) ? BLUE_HUB_CENTER : RED_HUB_CENTER;
        return hubCenter.minus(robotPose.getTranslation()).getNorm();
    }

    private static double getModuloRotation(double rawDeg) {
        double mod = rawDeg % 360.0;
        if (mod > 180.0)  mod -= 360.0;
        if (mod <= -180.0) mod += 360.0;
        return mod;
    }
}