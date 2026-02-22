package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX motor;
    private final VelocityVoltage velocityControl;
    private final StatusSignal<AngularVelocity> velocitySignal;

    private final Map<String, ShooterProfile> availableProfiles;
    private final SendableChooser<String> profileChooser;
    private ShooterProfile activeProfile;
    private String lastSelectedProfileName = "";

    private double targetRPM = 0.0;
    private boolean motorConfigured = false;

    private DCMotorSim m_motorSimModel;
    private TalonFXSimState m_talonFXSim;

    private double m_simMotorRPM = 0.0;

    public ShooterSubsystem() {
        motor = new TalonFX(ShooterConstants.MOTOR_ID, ShooterConstants.CANBUS);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = ShooterConstants.kP;
        config.Slot0.kI = ShooterConstants.kI;
        config.Slot0.kD = ShooterConstants.kD;
        config.Slot0.kS = ShooterConstants.kS;
        config.Slot0.kV = ShooterConstants.kV;
        config.Slot0.kA = ShooterConstants.kA;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = motor.getConfigurator().apply(config);
            if (status.isOK()) {
                motorConfigured = true;
                break;
            }
            Timer.delay(0.01);
        }

        if (!motorConfigured) {
            DriverStation.reportError(
                "Shooter motor (ID " + ShooterConstants.MOTOR_ID + ") config failed: " + status,
                false
            );
        }

        velocityControl = new VelocityVoltage(0).withSlot(0);
        velocitySignal = motor.getVelocity();
        velocitySignal.setUpdateFrequency(ShooterConstants.VELOCITY_UPDATE_FREQ);
        motor.optimizeBusUtilization();

        if (RobotBase.isSimulation()) {
            m_motorSimModel = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(1),  
                    0.001,                         
                    ShooterConstants.GEAR_RATIO
                ),
                DCMotor.getKrakenX60Foc(1)
            );
            m_talonFXSim = motor.getSimState();
            System.out.println("[ShooterSim] Physics model initialized.");
        }

        availableProfiles = ShooterConstants.createAllProfiles();
        profileChooser = new SendableChooser<>();

        boolean defaultSet = false;
        for (Map.Entry<String, ShooterProfile> entry : availableProfiles.entrySet()) {
            String profileName = entry.getKey();
            ShooterProfile profile = entry.getValue();

            if (profileName.equals(ShooterConstants.DEFAULT_PROFILE_NAME) && !defaultSet) {
                profileChooser.setDefaultOption(profile.getDisplayName(), profileName);
                defaultSet = true;
            } else {
                profileChooser.addOption(profile.getDisplayName(), profileName);
            }
        }

        SmartDashboard.putData("Shooter/Profile Selector", profileChooser);
        setActiveProfile(ShooterConstants.DEFAULT_PROFILE_NAME);

        DriverStation.reportWarning(
            "Shooter initialized with " + availableProfiles.size() + " profiles",
            false
        );
    }

    @Override
    public void periodic() {
        String selectedProfileName = profileChooser.getSelected();
        if (selectedProfileName != null && !selectedProfileName.equals(lastSelectedProfileName)) {
            setActiveProfile(selectedProfileName);
        }

        if (!motorConfigured) {
            SmartDashboard.putBoolean("Shooter/Motor Configured", false);
        }

        SmartDashboard.putNumber("Shooter/Target RPM", targetRPM);
        SmartDashboard.putNumber("Shooter/Actual RPM", getWheelRPM());
        SmartDashboard.putNumber("Shooter/Error RPM", targetRPM - getWheelRPM());
        SmartDashboard.putBoolean("Shooter/At Target", atTargetVelocity());
        SmartDashboard.putNumber("Shooter/Motor RPM", getMotorRPM());

        if (activeProfile != null) {
            SmartDashboard.putString("Shooter/Active Profile", activeProfile.getName());
            SmartDashboard.putNumber("Shooter/Profile Angle (deg)", activeProfile.getAngleDegrees());
            SmartDashboard.putNumber("Shooter/Profile Min Dist (m)", activeProfile.getMinSafeDistance());
            SmartDashboard.putNumber("Shooter/Profile Max Dist (m)", activeProfile.getMaxSafeDistance());
        }
    }

    public void simulationPeriodic() {
        if (m_motorSimModel == null || m_talonFXSim == null) return;

        m_talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        double motorVoltage = m_talonFXSim.getMotorVoltageMeasure().in(Volts);

        // Step 3 & 4: Apply to physics model and step forward
        m_motorSimModel.setInputVoltage(motorVoltage);
        m_motorSimModel.update(0.020);

        m_talonFXSim.setRawRotorPosition(
            m_motorSimModel.getAngularPosition().in(Rotations) * ShooterConstants.GEAR_RATIO
        );
        m_talonFXSim.setRotorVelocity(
            m_motorSimModel.getAngularVelocity().in(RotationsPerSecond) * ShooterConstants.GEAR_RATIO
        );

        double rotorRPS = m_motorSimModel.getAngularVelocity().in(RotationsPerSecond)
                          * ShooterConstants.GEAR_RATIO;
        m_simMotorRPM = rotorRPS * 60.0;

        SmartDashboard.putNumber("Shooter/Sim Motor RPM", m_simMotorRPM);
        SmartDashboard.putNumber("Shooter/Sim Wheel RPM", m_simMotorRPM / ShooterConstants.GEAR_RATIO);
        SmartDashboard.putNumber("Shooter/Sim Voltage", motorVoltage);
    }

    public void setVelocityForDistance(double distanceMeters) {
        if (activeProfile == null) {
            DriverStation.reportError("No active shooter profile", false);
            return;
        }
        double wheelRPM = getRPMForDistance(distanceMeters);
        setVelocityRPM(wheelRPM);
        SmartDashboard.putNumber("Shooter/Last Distance (m)", distanceMeters);
        SmartDashboard.putNumber("Shooter/Last Commanded RPM", wheelRPM);
    }

    public void setVelocityRPM(double wheelRPM) {
        targetRPM = wheelRPM;
        double motorRPM = wheelRPM * ShooterConstants.GEAR_RATIO;
        double motorRPS = motorRPM / 60.0;
        motor.setControl(velocityControl.withVelocity(motorRPS));
    }

    public void stop() {
        targetRPM = 0.0;
        motor.stopMotor();
    }

    /**
     * Returns wheel surface speed in RPM.
     * In simulation: reads from the DCMotorSim physics model.
     * On real robot: reads from the TalonFX velocity signal.
     */
    public double getWheelRPM() {
        if (RobotBase.isSimulation()) {
            return m_simMotorRPM / ShooterConstants.GEAR_RATIO;
        }
        return getMotorRPM() / ShooterConstants.GEAR_RATIO;
    }

    /**
     * Returns motor shaft speed in RPM.
     * In simulation: reads from the DCMotorSim physics model.
     * On real robot: reads from the TalonFX velocity signal.
     */
    public double getMotorRPM() {
        if (RobotBase.isSimulation()) {
            return m_simMotorRPM;
        }
        return velocitySignal.getValueAsDouble() * 60.0;
    }

    public boolean atTargetVelocity() {
        if (targetRPM == 0.0) return false;
        return Math.abs(targetRPM - getWheelRPM()) < ShooterConstants.VELOCITY_TOLERANCE_RPM;
    }

    public boolean isDistanceInRange(double distanceMeters) {
        if (activeProfile == null) return false;
        return activeProfile.isDistanceInRange(distanceMeters);
    }

    public String getActiveProfileName() {
        return activeProfile != null ? activeProfile.getName() : "NONE";
    }

    public double getActiveProfileAngle() {
        return activeProfile != null ? activeProfile.getAngleDegrees() : 0.0;
    }

    public void setActiveProfile(String profileName) {
        if (!availableProfiles.containsKey(profileName)) {
            DriverStation.reportError("Profile '" + profileName + "' not found. Using default.", false);
            profileName = ShooterConstants.DEFAULT_PROFILE_NAME;
        }
        activeProfile = availableProfiles.get(profileName);
        lastSelectedProfileName = profileName;
        DriverStation.reportWarning(
            String.format("Shooter profile: %s (%.1f°, %.1f-%.1fm)",
                activeProfile.getName(),
                activeProfile.getAngleDegrees(),
                activeProfile.getMinSafeDistance(),
                activeProfile.getMaxSafeDistance()),
            false
        );
    }

    private double getRPMForDistance(double distance) {
        if (activeProfile == null) {
            DriverStation.reportError("No active profile - using default RPM", false);
            return 3500.0;
        }
        if (distance < activeProfile.getMinSafeDistance()) {
            DriverStation.reportWarning(
                String.format("Distance %.2fm below min %.2fm - using edge value",
                    distance, activeProfile.getMinSafeDistance()), false);
            SmartDashboard.putBoolean("Shooter/Distance In Range", false);
            return activeProfile.getRPMForDistance(activeProfile.getMinSafeDistance());
        }
        if (distance > activeProfile.getMaxSafeDistance()) {
            DriverStation.reportWarning(
                String.format("Distance %.2fm exceeds max %.2fm - using edge value",
                    distance, activeProfile.getMaxSafeDistance()), false);
            SmartDashboard.putBoolean("Shooter/Distance In Range", false);
            return activeProfile.getRPMForDistance(activeProfile.getMaxSafeDistance());
        }
        SmartDashboard.putBoolean("Shooter/Distance In Range", true);
        return activeProfile.getRPMForDistance(distance);
    }
}