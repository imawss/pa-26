package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import frc.robot.constants.IntakeConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax intakeMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;

    private double targetPosition = IntakeConstants.INTAKE_RETRACTED_DEGREES;
    private boolean manualMode = false;
    private boolean holdingPosition = false;

    private int atTargetCount = 0;
    private int driftedCount  = 0;
    private static final int DEBOUNCE_LOOPS = 3;

    private int stallLoopCount = 0;
    private static final int STALL_LOOP_THRESHOLD = 50; 

    public IntakeSubsystem() {
        intakeMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake)
              .smartCurrentLimit(30)
              .inverted(false);

        double positionConversionFactor = 360.0 / IntakeConstants.GEAR_RATIO;
        config.encoder
              .positionConversionFactor(positionConversionFactor)
              .velocityConversionFactor(positionConversionFactor / 60.0);

        config.closedLoop
              .pid(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD)
              .positionWrappingEnabled(false);

        config.softLimit
              .forwardSoftLimit(IntakeConstants.INTAKE_EXTENDED_DEGREES - 5)
              .forwardSoftLimitEnabled(true)
              .reverseSoftLimit(IntakeConstants.INTAKE_RETRACTED_DEGREES + 5)
              .reverseSoftLimitEnabled(true);

        config.signals
              .outputCurrentPeriodMs(20)
              .appliedOutputPeriodMs(20)
              .busVoltagePeriodMs(20);

        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor.clearFaults();

        encoder = intakeMotor.getEncoder();
        pidController = intakeMotor.getClosedLoopController();

        encoder.setPosition(IntakeConstants.INTAKE_RETRACTED_DEGREES);
    }

    public void extend() {
        setTargetPosition(IntakeConstants.INTAKE_EXTENDED_DEGREES);
    }

    public void retract() {
        setTargetPosition(IntakeConstants.INTAKE_RETRACTED_DEGREES);
    }

    public void toggle() {
        if (Math.abs(targetPosition - IntakeConstants.INTAKE_RETRACTED_DEGREES)
                < IntakeConstants.POSITION_TOLERANCE) {
            extend();
        } else {
            retract();
        }
    }

    public void setSpeed(double speed) {
        manualMode = true;
        holdingPosition = false;
        stallLoopCount = 0;
        intakeMotor.set(MathUtil.clamp(speed, -1.0, 1.0));
        targetPosition = encoder.getPosition();
    }

    public void stop() {
        targetPosition = encoder.getPosition();
        manualMode = false;
        holdingPosition = false;
        stallLoopCount = 0;
        atTargetCount  = 0;
        driftedCount   = 0;
        pidController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void calibrate() {
        if (!isRetracted() && !manualMode) {
            System.out.println("[IntakeSubsystem] WARNING: calibrate() rejected — intake is not retracted. " +
                               "Retract fully before calibrating.");
            return;
        }
        intakeMotor.stopMotor();
        encoder.setPosition(IntakeConstants.INTAKE_RETRACTED_DEGREES);
        targetPosition    = IntakeConstants.INTAKE_RETRACTED_DEGREES;
        manualMode        = false;
        holdingPosition   = true; 
        stallLoopCount    = 0;
        atTargetCount     = 0;
        driftedCount      = 0;
        System.out.println("[IntakeSubsystem] Encoder calibrated to "
                           + IntakeConstants.INTAKE_RETRACTED_DEGREES + " degrees");
    }

    public boolean isExtended() {
        return Math.abs(encoder.getPosition() - IntakeConstants.INTAKE_EXTENDED_DEGREES)
               < IntakeConstants.POSITION_TOLERANCE;
    }

    public boolean isRetracted() {
        return Math.abs(encoder.getPosition() - IntakeConstants.INTAKE_RETRACTED_DEGREES)
               < IntakeConstants.POSITION_TOLERANCE;
    }

    public boolean atTarget() {
        boolean positionOk = Math.abs(encoder.getPosition() - targetPosition)
                             < IntakeConstants.POSITION_TOLERANCE;
        boolean velocityOk = Math.abs(getVelocity()) < IntakeConstants.SETTLED_VELOCITY_THRESHOLD;
        return positionOk && velocityOk;
    }

    public double getPosition()       { return encoder.getPosition(); }
    public double getVelocity()       { return encoder.getVelocity(); }
    public double getTargetPosition() { return targetPosition; }
    public double getMotorCurrent()   { return intakeMotor.getOutputCurrent(); }
    public boolean isManualMode()     { return manualMode; }

    @Deprecated
    public void resetEncoder() {
        throw new UnsupportedOperationException(
            "resetEncoder() is removed. Call calibrate() instead.");
    }

    private void setTargetPosition(double positionDeg) {
        holdingPosition = false;
        manualMode      = false;
        stallLoopCount  = 0;
        atTargetCount   = 0;
        driftedCount    = 0;
        targetPosition  = positionDeg;
        pidController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void periodic() {
        boolean isStalled = !manualMode && !holdingPosition &&
                            Math.abs(getVelocity()) < IntakeConstants.STALL_VELOCITY_THRESHOLD &&
                            Math.abs(targetPosition - getPosition()) > IntakeConstants.POSITION_TOLERANCE;

        if (isStalled) {
            stallLoopCount++;
            if (stallLoopCount >= STALL_LOOP_THRESHOLD) {
                intakeMotor.stopMotor();
                holdingPosition = true; 
                System.out.println("[IntakeSubsystem] STALL DETECTED — motor stopped to prevent damage.");
            }
        } else {
            stallLoopCount = 0;
        }

        if (!manualMode && !holdingPosition && atTarget()) {
            atTargetCount++;
            if (atTargetCount >= DEBOUNCE_LOOPS) {
                intakeMotor.stopMotor();
                holdingPosition = true;
                atTargetCount   = 0;
                driftedCount    = 0;
            }
        } else {
            atTargetCount = 0;
        }

        if (!manualMode && holdingPosition && !atTarget() && !isStalled) {
            driftedCount++;
            if (driftedCount >= DEBOUNCE_LOOPS) {
                holdingPosition = false;
                stallLoopCount  = 0;
                driftedCount    = 0;
                atTargetCount   = 0;
                pidController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
            }
        } else if (!holdingPosition) {
            driftedCount = 0;
        }

        SmartDashboard.putNumber("Intake/Position (deg)",       getPosition());
        SmartDashboard.putNumber("Intake/Target Position (deg)", targetPosition);
        SmartDashboard.putNumber("Intake/Velocity (deg/s)",     getVelocity());
        SmartDashboard.putNumber("Intake/Position Error",       targetPosition - getPosition());
        SmartDashboard.putBoolean("Intake/Is Extended",         isExtended());
        SmartDashboard.putBoolean("Intake/Is Retracted",        isRetracted());
        SmartDashboard.putBoolean("Intake/At Target",           atTarget());
        SmartDashboard.putBoolean("Intake/Holding Position",    holdingPosition);
        SmartDashboard.putNumber("Intake/Motor Current",        getMotorCurrent());
        SmartDashboard.putNumber("Intake/Applied Output",       intakeMotor.getAppliedOutput());
        SmartDashboard.putNumber("Intake/Bus Voltage",          intakeMotor.getBusVoltage());
        SmartDashboard.putBoolean("Intake/Manual Mode",         manualMode);
        SmartDashboard.putBoolean("Intake/High Current Warning",getMotorCurrent() > 25.0);
        SmartDashboard.putBoolean("Intake/Stalled",             isStalled);
        SmartDashboard.putNumber("Intake/Stall Loop Count",     stallLoopCount);

        boolean outOfBounds = getPosition() < (IntakeConstants.INTAKE_RETRACTED_DEGREES - 15) ||
                              getPosition() > (IntakeConstants.INTAKE_EXTENDED_DEGREES + 15);
        SmartDashboard.putBoolean("Intake/Out of Bounds", outOfBounds);
    }
}