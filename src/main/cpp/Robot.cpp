#define PI 3.14159265358979323846

#include <frc/TimedRobot.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/XboxController.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Rotation2d.h>
#include <studica/AHRS.h>
#include <frc/SerialPort.h>
#include <frc/SPI.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/SparkMax.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/AnalogEncoder.h>
#include <cameraserver/CameraServer.h>
#include <frc/Timer.h>
#include <units/velocity.h>
#include <frc/BuiltInAccelerometer.h>
#include <frc/PneumaticsModuleType.h>
#include <rev/SparkClosedLoopController.h>
#include <frc/PneumaticHub.h>
#include <frc/DoubleSolenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <networktables/NetworkTable.h>

class Robot : public frc::TimedRobot
{
    // Defines the locations of each wheel
    frc::Translation2d m_frontLeftLocation{0.314_m, 0.314_m};
    frc::Translation2d m_frontRightLocation{0.314_m, -0.314_m};
    frc::Translation2d m_backLeftLocation{-0.314_m, 0.314_m};
    frc::Translation2d m_backRightLocation{-0.314_m, -0.314_m};

//    frc::Translation2d m_frontLeftLocation{0.314_m, -0.314_m};
//    frc::Translation2d m_frontRightLocation{-0.314_m, -0.314_m};
//    frc::Translation2d m_backLeftLocation{0.314_m, 0.314_m};
//    frc::Translation2d m_backRightLocation{-0.314_m, 0.314_m};

    // Setting up drive motors using odd CAN bus ID's
    rev::spark::SparkMax wheelfl{1, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController pidfl = wheelfl.GetClosedLoopController();
    rev::spark::SparkMax wheelfr{5, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController pidfr = wheelfr.GetClosedLoopController();
    rev::spark::SparkMax wheelbl{3, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController pidbl = wheelbl.GetClosedLoopController();
    rev::spark::SparkMax wheelbr{7, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController pidbr = wheelbr.GetClosedLoopController();

    // Configuration objects for PID control of motors via SparkMax's
    rev::spark::SparkBaseConfig driveConfig{};
    rev::spark::SparkBaseConfig steerConfig{};
    rev::spark::SparkBaseConfig elev1Config{};
    rev::spark::SparkBaseConfig otherConfig{};
    rev::spark::SparkBaseConfig hangConfig{};

    // for encoders, consider changing methods to GetAlternateEncoder with AlternateEncoder::Type::kHallEffect or something if you face an error
    // Setting up steering motors using even CAN bus ID's
    // The constructor parameter is the "analog input channel" to use, corresponding to
    // the RoboRIO AnalogIn pins.
    rev::spark::SparkMax rotfl{2, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController rotpidfl = rotfl.GetClosedLoopController();
    frc::AnalogEncoder encfl{3};
    rev::spark::SparkMax rotfr{6, rev::spark::SparkLowLevel::MotorType::kBrushless};
    frc::AnalogEncoder encfr{2};
    rev::spark::SparkMax rotbl{4, rev::spark::SparkLowLevel::MotorType::kBrushless};
    frc::AnalogEncoder encbl{0};
    rev::spark::SparkMax rotbr{8, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkRelativeEncoder m_brSteerEnc = rotbr.GetEncoder();
    frc::AnalogEncoder encbr{1};

    // Elevator motors
    rev::spark::SparkMax elev1{20, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController elev1pid = elev1.GetClosedLoopController();
    rev::spark::SparkMax elev2{21, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController elev2pid = elev2.GetClosedLoopController();
    rev::spark::SparkMax elev3{22, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController elev3pid = elev3.GetClosedLoopController();

    // Algae motors
    rev::spark::SparkMax alg1{30, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController alg1pid = alg1.GetClosedLoopController();
    rev::spark::SparkMax alg2{31, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController alg2pid = alg2.GetClosedLoopController();

    // Hang motor
    rev::spark::SparkMax hang{40, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController hangpid = hang.GetClosedLoopController();

    int abuttoncount = 0;
    double d = 180;
    double speedfactor = 2000;
    double getto = 3000;
    double changing = false;
    double cfac = 0;

    frc::Timer time;

    frc::BuiltInAccelerometer acc;
    // AHRS: attitude and heading reference system
    studica::AHRS *ahrs = new studica::AHRS(studica::AHRS::NavXComType::kUSB1);
    // Creating my kinematics object using the module locations.
    frc::SwerveDriveKinematics<4> kinematics{
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
        m_backRightLocation};
    frc::XboxController controller{0};
    frc::XboxController controller2{1};
    frc::SlewRateLimiter<units::meters_per_second> limitx{9_mps / .5_s};
    frc::SlewRateLimiter<units::meters_per_second> limity{9_mps / .5_s};

    //Controller Mode Variables
    bool m_manual_mode = true;
    
    void RobotInit()
    {
        // This section, taken from the REVLib 2025 documentation, uses C++ "method chaining."
        // This is a compact notation whereby multiple calls on a single object can be combined,
        // so long as the methods return a reference to the object itself (as these methods do).
        // This is why all but the final method call in such chained calls omit the semicolon
        // at the end of their respective lines.
        driveConfig
            .Inverted(true)
            .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
        driveConfig.encoder
            .PositionConversionFactor(1)
            .VelocityConversionFactor(1);
            // The following conversion factors caused mayhem for 2025:
            // .PositionConversionFactor(2 * PI * 2.0 * 0.0254)
            // .VelocityConversionFactor(2 * PI * 2.0 * 0.0254 / 60);
        driveConfig.closedLoop
            .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
            .Pid(0.0001, 0.000001, 0.00000001)
            .IZone(4000);

        steerConfig
            .Inverted(true)
            .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
        steerConfig.encoder
            .PositionConversionFactor(1)
            .VelocityConversionFactor(1);
            // These caused mayhem for 2025:
            // .PositionConversionFactor(2 * PI)
            // .VelocityConversionFactor(2 * PI / 60);
        steerConfig.closedLoop
            .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
            .Pid(0.0001, 0.000001, 0.00000001)
            .IZone(4000);

        wheelfl.Configure(driveConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
            rev::spark::SparkMax::PersistMode::kPersistParameters);
        wheelfr.Configure(driveConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
            rev::spark::SparkMax::PersistMode::kPersistParameters);
        wheelbl.Configure(driveConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
            rev::spark::SparkMax::PersistMode::kPersistParameters);
        wheelbr.Configure(driveConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
            rev::spark::SparkMax::PersistMode::kPersistParameters);

        rotfl.Configure(steerConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
            rev::spark::SparkMax::PersistMode::kPersistParameters);
        rotfr.Configure(steerConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
            rev::spark::SparkMax::PersistMode::kPersistParameters);
        rotbl.Configure(steerConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
            rev::spark::SparkMax::PersistMode::kPersistParameters);
        rotbr.Configure(steerConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
            rev::spark::SparkMax::PersistMode::kPersistParameters);

        wheelfl.GetEncoder().SetPosition(0);
        rotfl.GetEncoder().SetPosition(encfl.Get());

        elev1Config
            .Inverted(true)
            .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
        elev1Config.encoder
            .PositionConversionFactor(1)
            .VelocityConversionFactor(1);
        elev1Config.closedLoop
            .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
            .Pid(0.025, 0.000002, 0.00000001)
            .IZone(4000);

        otherConfig
            .Inverted(true)
            .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
        otherConfig.encoder
            .PositionConversionFactor(1)
            .VelocityConversionFactor(1);
        otherConfig.closedLoop
            .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
            .Pid(0.0275, 0.000002, 0.00000001)
            .IZone(4000);

        hangConfig
            .Inverted(true)
            .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
        hangConfig.encoder
            .PositionConversionFactor(1)
            .VelocityConversionFactor(1);
        hangConfig.closedLoop
            .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
            .Pid(0.0001, 0.0, 0.0)
            .IZone(4000);

        elev1.Configure(elev1Config, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
            rev::spark::SparkMax::PersistMode::kPersistParameters);
        elev1.GetEncoder().SetPosition(0);
        elev2.Configure(otherConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
            rev::spark::SparkMax::PersistMode::kPersistParameters);
        elev2.GetEncoder().SetPosition(0);
        elev3.Configure(otherConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
            rev::spark::SparkMax::PersistMode::kPersistParameters);

        alg1.Configure(otherConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
            rev::spark::SparkMax::PersistMode::kPersistParameters);
        alg2.Configure(otherConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
            rev::spark::SparkMax::PersistMode::kPersistParameters);

        hang.Configure(hangConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters,
            rev::spark::SparkMax::PersistMode::kPersistParameters);

        ahrs->Reset();
        ahrs->ResetDisplacement();
        ahrs->SetAngleAdjustment(0);
        
        // Camera
        frc::CameraServer::StartAutomaticCapture();
    }

    void RobotPeriodic() {}

    void AutonomousInit()
    {
        time.Reset();
        time.Start();
    }

    void TeleopInit()
    {
        time.Stop();
        time.Reset();
        speedfactor = 2000;
    }

    void AutonomousPeriodic()
    {
        // Autonomous is 15 seconds long

        // First, drive for 4 seconds to the reef:
        if (time.Get() <= 4_s)
        {
            Drive(0.0, 0.3, 0);
        }
        else
        {
            // Stop driving:
            Drive(0, 0, 0);

            // Go to L1 position:
            if (time.Get() < 6_s)
            {
                elev1pid.SetReference(3, rev::spark::SparkBase::ControlType::kPosition);
                elev2pid.SetReference(20, rev::spark::SparkBase::ControlType::kPosition);
            }
            else if (time.Get() < 8_s)
            {
                // Maintain L1 position and shoot:
                elev1pid.SetReference(3, rev::spark::SparkBase::ControlType::kPosition);
                elev2pid.SetReference(20, rev::spark::SparkBase::ControlType::kPosition);
                elev3.Set(-0.5);
            }
            else
            {
                // Return to rest position and stop shooting:
                elev1pid.SetReference(0, rev::spark::SparkBase::ControlType::kPosition);
                elev2pid.SetReference(0, rev::spark::SparkBase::ControlType::kPosition);
                elev3.Set(0.0);
            }
        }

        /*
        else if (time.Get() > 5_s && time.Get() < 7_s)
        {
            Drive(0, 0, 0);
            ahrs->ResetDisplacement();
        }
        else if (fabs(ahrs->GetDisplacementY()) < 2.1 && time.Get() < 15_s)
        {
            Drive(0.15, 0, 0);
        }
        else
        {
            Drive(0, 0, 0);
        }
        */

        return;

        if (time.Get() <= 5_s)
        {
            /*
            // spin shooter
            pidshoot_top.SetReference(10000,  rev::spark::SparkBase::ControlType::kVelocity);
            pidshoot_top.SetIAccum(0);
            pidshoot_bottom.SetReference(10000,  rev::spark::SparkBase::ControlType::kVelocity);
            pidshoot_bottom.SetIAccum(0);
            */
        }
/*
        else if ((time.Get() > 5_s) && (time.Get() < 8_s))
        {
            // stop shooter
            pidshoot_top.SetReference(0,  rev::spark::SparkBase::ControlType::kVelocity);
            pidshoot_top.SetIAccum(0);
            pidshoot_bottom.SetReference(0,  rev::spark::SparkBase::ControlType::kVelocity);
            pidshoot_bottom.SetIAccum(0);

            // drive forward 1.2 meters
            Drive(0, -0.1, 0);
        }
        else
        {
            Drive(0, 0, 0);
        }
*/
        else
        {
            /*
            // stop shooter
            pidshoot_top.SetReference(0,  rev::spark::SparkBase::ControlType::kVelocity);
            pidshoot_top.SetIAccum(0);
            pidshoot_bottom.SetReference(0,  rev::spark::SparkBase::ControlType::kVelocity);
            pidshoot_bottom.SetIAccum(0);
            */

            if (fabs(ahrs->GetDisplacementY()) < 2.54)
            {
//                AutoDrive(0, 0.5, 0);
                Drive(0, -0.4, 0);
            }
            else
            {
                Drive(0, 0, 0);
            }
        }
    }

    void Drive(double x, double y, double rotate)
    {
        // if (x != 0 && y != 0 && rotate != 0) {
        try
        {
            d = 360 - ahrs->GetAngle();
            frc::SmartDashboard::PutString("connection", "connected");
        }
        catch (int degree)  // This value is not used?
        {
            frc::SmartDashboard::PutString("connection", "lost");
        }
        units::degree_t degr{d};
        frc::Rotation2d rot2d{degr};  // OK, rot2d reflects the AHRS orientation...

        frc::SmartDashboard::PutNumber("Drive:x", x);
        frc::SmartDashboard::PutNumber("Drive:y", y);
        frc::SmartDashboard::PutNumber("Drive:rotate", rotate);
        frc::SmartDashboard::PutNumber("Drive: AHRS (rot2d)", rot2d.Degrees().value());
        frc::SmartDashboard::PutNumber("encfl.Get", encfl.Get());
        frc::SmartDashboard::PutNumber("encfr.Get", encfr.Get());
        frc::SmartDashboard::PutNumber("encbl.Get", encbl.Get());
        frc::SmartDashboard::PutNumber("encbr.Get", encbr.Get());

        if (rotate * speedfactor > 4000)
        {
            rotate = 4000 / speedfactor;
        }
        else if (rotate * speedfactor * (-1) > 4000)
        {
            rotate = -4000 / speedfactor;
        }

        units::radians_per_second_t rad{rotate};
        units::meters_per_second_t speedy{y};
        units::meters_per_second_t speedx{x};
        frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            limitx.Calculate(speedx),
            limity.Calculate(speedy),
            rad * 1.2,  // * 1.2 ??
            rot2d  // This is what enables field-oriented control
        );

        auto [fl, fr, bl, br] = kinematics.ToSwerveModuleStates(speeds);

        // move swerve motors to angles here (i.e. the STEERING angle)
        double getfl = fl.angle.Radians().value() / 2.0 / PI;  // i.e. number of rotations
        double getfr = fr.angle.Radians().value() / 2.0 / PI;
        double getbl = bl.angle.Radians().value() / 2.0 / PI;
        double getbr = br.angle.Radians().value() / 2.0 / PI;

        // These values pertain to the DRIVING speed
        double frole = fl.speed.value();
        double frori = fr.speed.value();
        double bale = bl.speed.value();
        double bari = br.speed.value();

        frc::SmartDashboard::PutNumber("kinematics: FL.angle", getfl);
        frc::SmartDashboard::PutNumber("kinematics: FR.angle", getfr);
        frc::SmartDashboard::PutNumber("kinematics: BL.angle", getbl);
        frc::SmartDashboard::PutNumber("kinematics: BR.angle", getbr);

        frc::SmartDashboard::PutNumber("kinematics: FL.speed", frole);
        frc::SmartDashboard::PutNumber("kinematics: FR.speed", frori);
        frc::SmartDashboard::PutNumber("kinematics: BL.speed", bale);
        frc::SmartDashboard::PutNumber("kinematics: BR.speed", bari);

        double flpos = fmod(encfl.Get() + 0.0, 1) - getfl;  // steering??

        // Adjust flpos so that it is in the range [-0.5, 0.5]?
        if (flpos > 0.5)
        {
            flpos -= 1;
        }
        else if (flpos < -0.5)
        {
            flpos += 1;
        }
        double frpos = fmod(encfr.Get() - 0.36, 1) - getfr;
        if (frpos > 0.5)
        {
            frpos -= 1;
        }
        else if (frpos < -0.5)
        {
            frpos += 1;
        }
        double blpos = fmod(encbl.Get() + 0.395, 1) - getbl;
        if (blpos > 0.5)
        {
            blpos -= 1;
        }
        else if (blpos < -0.5)
        {
            blpos += 1;
        }
        double brpos = fmod(encbr.Get() - 0.01, 1) - getbr;
        if (brpos > 0.5)
        {
            brpos -= 1;
        }
        else if (brpos < -0.5)
        {
            brpos += 1;
        }

        frc::SmartDashboard::PutNumber("Drive: flpos", flpos);
        frc::SmartDashboard::PutNumber("Drive: frpos", frpos);
        frc::SmartDashboard::PutNumber("Drive: blpos", blpos);
        frc::SmartDashboard::PutNumber("Drive: brpos", brpos);

        rotfl.Set(-flpos * 1.5);
        rotfr.Set(-frpos * 1.5);
        rotbl.Set(-blpos * 1.5);
        rotbr.Set(-brpos * 1.5);

        frc::SmartDashboard::PutNumber("FL vel", rotfl.GetEncoder().GetVelocity());
        frc::SmartDashboard::PutNumber("FR vel", rotfr.GetEncoder().GetVelocity());
        frc::SmartDashboard::PutNumber("BL vel", rotbl.GetEncoder().GetVelocity());
        frc::SmartDashboard::PutNumber("BR vel", rotbr.GetEncoder().GetVelocity());

        pidfl.SetReference(frole * speedfactor,  rev::spark::SparkBase::ControlType::kVelocity);
        pidfr.SetReference(frori * speedfactor,  rev::spark::SparkBase::ControlType::kVelocity);
        pidbl.SetReference(bale * speedfactor,  rev::spark::SparkBase::ControlType::kVelocity);
        pidbr.SetReference(bari * speedfactor,  rev::spark::SparkBase::ControlType::kVelocity);

        return;
    }

     void Drive2(double x, double y, double rotate)
    {
        d = 360 - ahrs->GetAngle();
        units::degree_t degr{d};
        frc::Rotation2d rot2d{degr};  // OK, rot2d reflects the AHRS orientation...

        units::meters_per_second_t speedx{x};
        units::meters_per_second_t speedy{y};
        units::radians_per_second_t rad{rotate};
        frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            limitx.Calculate(speedx),
            limity.Calculate(speedy),
            rad,
            rot2d  // This is what enables field-oriented control
        );

        auto [fl, fr, bl, br] = kinematics.ToSwerveModuleStates(speeds);

        SetDesiredStates(fl, fr, bl, br);

        return;

        units::radian_t encflrad{encfl.Get() * 2 * PI};
        frc::Rotation2d flcurrangle{encflrad};
        auto flOptimized = frc::SwerveModuleState::Optimize(fl, flcurrangle);
        flOptimized.speed *= (flOptimized.angle - flcurrangle).Cos();

        double setangle = flOptimized.angle.Radians().value() * 0.01;
        setangle = fmin(setangle, 1.0);
        setangle = fmax(setangle, -1.0);
        rotfl.Set(setangle);

        double setspeed = flOptimized.speed.value() * 0.01;
        setspeed = fmin(setspeed, 1.0);
        setspeed = fmax(setspeed, -1.0);
        pidfl.SetReference(setspeed, rev::spark::SparkBase::ControlType::kVelocity);

        return;
    }

   void TeleopPeriodic()
    {
        frc::SmartDashboard::PutNumber("TeleopPeriodic: wheelfl.Get()", wheelfl.Get());  // wheelfl.Get() returns the current set speed in [-1,1]
        frc::SmartDashboard::PutNumber("TeleopPeriodic: wheelfr.Get()", wheelfr.Get());
        frc::SmartDashboard::PutNumber("TeleopPeriodic: wheelbl.Get()", wheelbl.Get());
        frc::SmartDashboard::PutNumber("TeleopPeriodic: wheelbr.Get()", wheelbr.Get());

        // ------------
        // CONTROLLER 1
        // ------------

        if (controller.GetStartButtonPressed())
        {
            m_manual_mode = !m_manual_mode;
        }
        
        if (m_manual_mode)
        {
            double con1RT = controller.GetRightTriggerAxis();
            double con1LT = controller.GetLeftTriggerAxis();
            if ((con1RT > 0.1) && !(con1LT > 0.1))
            {
                // Raise elevator
                elev1.Set(0.5 * con1RT);  // We could try SetReference on the closed loop controller with kPosition
                elev1pid.SetIAccum(0);
            }
            else if (!(con1RT > 0.1) && (con1LT > 0.1))
            {
                // Lower elevator
                elev1.Set(-0.5 * con1LT);
                elev1pid.SetIAccum(0);
            }
            else if (!(con1RT > 0.1) && !(con1LT > 0.1))
            {
                // Stop elevator
                elev1.Set(0.0);
                elev1pid.SetIAccum(0);
            }

            // Coral shoulder up and down are mutually exclusive
            if (controller.GetAButton() && !controller.GetBButton())
            {
                elev2.Set(0.3);
                elev2pid.SetIAccum(0);
            }
            else if (!controller.GetAButton() && controller.GetBButton())
            {
                elev2.Set(-0.3);
                elev2pid.SetIAccum(0);
            }
            else
            {
                elev2.Set(0.0);
                elev2pid.SetIAccum(0);
            }
        }

        if (controller.GetYButton())
        {
            pidfl.SetIAccum(0);
            pidfr.SetIAccum(0);
            pidbl.SetIAccum(0);
            pidbr.SetIAccum(0);
            elev1pid.SetIAccum(0);
            elev2pid.SetIAccum(0);
            hangpid.SetIAccum(0);
        }
        
        // Coral in and coral out are mutually exclusive
        if (controller.GetRightBumper() && !controller.GetLeftBumper())
        {
            // Coral positive
            elev3.Set(0.2);
            elev3pid.SetIAccum(0);
        }
        else if (!controller.GetRightBumper() && controller.GetLeftBumper())
        {
            // Coral negative
            elev3.Set(-0.5);
            elev3pid.SetIAccum(0);
        }
        else
        {
            // Stop coral
            elev3.Set(0.0);
            elev3pid.SetIAccum(0);
        }

        // xstop and Drive are mutually exclusive
        if (controller.GetXButton())
        {
            xstop();
            pidfl.SetIAccum(0);
            pidfr.SetIAccum(0);
            pidbl.SetIAccum(0);
            pidbr.SetIAccum(0);
        } 
        else
        {   
            if (controller.GetLeftStickButton() && !controller.GetRightStickButton())
            {
                if (speedfactor < 6000)
                {
                    speedfactor += 20;
                }
            }
            else if (!controller.GetLeftStickButton() && controller.GetRightStickButton())
            {
                if (speedfactor > 1000)
                {
                    speedfactor -= 20;
                }
            }
    
            double drift = 0.1;
            double x = controller.GetLeftY();  // Assigning joystick Y to field X
            if (x < drift && x > -drift)
            {
                x = 0;
            }

            double y = controller.GetLeftX();  // Assigning joystick X to field Y
            if (y < drift && y > -drift)
            {
                y = 0;
            }

            double turn = controller.GetRightX();
            if (turn < drift && turn > -drift)
            {
                turn = 0;
            }

            // absolute encoders are analog and measure position in rotations
            // We're calling Drive with values taken (pretty much) directly from the joysticks
            // Also, even though the field defines plus-x as forward, the joysticks don't, so
            // joystick up is joystick Y is field X. Nice...
            Drive(x, y, turn);
        }

        // controller2
        // to accelerate

        /*
        int shoot_speed = 0;
        int add_value = 100;
        int speed_cap = 10000;
        int accelerate = 1;
        while (accelerate == 1);
        {
        */

        // ------------
        // CONTROLLER 2
        // ------------
        if (controller2.GetRightBumper() && !controller2.GetLeftBumper())
        {
            // Engage hang
//            hang.Set(0.7);
            hangpid.SetReference(0.1, rev::spark::SparkBase::ControlType::kVelocity);
        }
        else if (!controller2.GetRightBumper() && controller2.GetLeftBumper())
        {
            // Disengage hang
//            hang.Set(-0.7);
            hangpid.SetReference(-0.1, rev::spark::SparkBase::ControlType::kVelocity);
        }
        else
        {
            // Stop hang...
//            hang.Set(0.0);
            hangpid.SetReference(0.0, rev::spark::SparkBase::ControlType::kVelocity);

            // ... and check for other conditions
            if (controller2.GetXButton() && !controller2.GetBButton())
            {
                // Rotate algae arm positive
                alg1.Set(0.05);
                alg1pid.SetIAccum(0);
            }
            else if (!controller2.GetXButton() && controller2.GetBButton())
            {
                // Rotate algae arm negative
                alg1.Set(-0.1);
                alg1pid.SetIAccum(0);
            }
            else
            {
                // Stop algae arm rotation
                alg1.Set(0.0);
                alg1pid.SetIAccum(0);
            }

            double con2RT = controller2.GetRightTriggerAxis();
            double con2LT = controller2.GetLeftTriggerAxis();
            if ((con2RT > 0.1) && !(con2LT > 0.1))
            {
                // Drive algae roller positive
                alg2.Set(-0.3 * con2RT);
                alg2pid.SetIAccum(0);
            }
            else if (!(con2RT > 0.1) && (con2LT > 0.1))
            {
                // Drive algae roller negative
                alg2.Set(0.3 * con2LT);
                alg2pid.SetIAccum(0);
            }
            else if (!(con2RT > 0.1) && !(con2LT > 0.1))
            {
                // Stop algae roller
                alg2.Set(0.0);
                alg2pid.SetIAccum(0);
            }
            if (!m_manual_mode)
            {
                // Coral Station Preset
                if (controller2.GetLeftY() >= 0.75)
                {
                    elev1pid.SetReference(39, rev::spark::SparkBase::ControlType::kPosition);
                    elev2pid.SetReference(34, rev::spark::SparkBase::ControlType::kPosition);
                }

                // L1 Preset
                else if (controller2.GetLeftY() <= -0.75)
                {
                    elev1pid.SetReference(3, rev::spark::SparkBase::ControlType::kPosition);
                    elev2pid.SetReference(20, rev::spark::SparkBase::ControlType::kPosition);
                }

                // L2 Preset
                else if (controller2.GetRightY() >= 0.75)
                {
                    elev1pid.SetReference(49, rev::spark::SparkBase::ControlType::kPosition);
                    elev2pid.SetReference(75, rev::spark::SparkBase::ControlType::kPosition);
                }

                // L3 Preset
                else if (controller2.GetRightY() <= -0.75)
                {
                    elev1pid.SetReference(77, rev::spark::SparkBase::ControlType::kPosition);
                    elev2pid.SetReference(75, rev::spark::SparkBase::ControlType::kPosition);
                }

                // Home
                else
                {
                    elev1pid.SetReference(3, rev::spark::SparkBase::ControlType::kPosition);
                    elev2pid.SetReference(0, rev::spark::SparkBase::ControlType::kPosition);
                }
                //else if L1
                //else if L2
                //else if L3
                //else Home
            }
        }
        frc::SmartDashboard::PutNumber("Elev1 pos", elev1.GetEncoder().GetPosition());
        frc::SmartDashboard::PutNumber("Elev2 pos", elev2.GetEncoder().GetPosition());
    }

    void xstop()
    {
        // move swerve motors to angles here
        double getfl = .125;
        double getfr = .875;
        double getbl = .875;
        double getbr = .125;

        // Assuming that the absolute steering encoders (e.g. encfl) return rotations
        // in response to Get(), then all of these values are in rotations (and I 
        // wonder if 0.825 is supposed to be 0.875). So we obtain the current rotational
        // position and subtract the desired angle (i.e. 1/8 of a rotation, or 3/8 of a
        // rotation). If the result is more than half a rotation, then there's a more
        // efficient way to get to that position by rotating in the opposite direction,
        // hence the -= 1 or +=1.

        // The difference between the absolute angular position and the desired position
        // (e.g. 0.125) is simply the relative turn that the wheel needs to make to get
        // to where we want it to go. Presumably when we use this difference, i.e. the
        // relative turning angle to make, to call the *relative* encoder rotfl, we're
        // asking for a turn *relative* to the current position.

        // Questions:
        // 1. Do we ever zero-out the encoders, or otherwise try to normalize them to
        //    some known starting angle?
        // 2. Why for the corners other than FL do we use the fmod function?
        // 3. Why do we multiply the pos values by 1.5?
        // 4. Suppose encfl.Get() returns the absolute angle of the wheel... 
        double flpos = fmod(encfl.Get() + 0.0, 1) - getfl;
        if (flpos > 0.5)
        {
            flpos -= 1;
        }
        else if (flpos < -0.5)
        {
            flpos += 1;
        }

        double frpos = fmod(encfr.Get() - 0.36, 1) - getfr;
        if (frpos > 0.5)
        {
            frpos -= 1;
        }
        else if (frpos < -0.5)
        {
            frpos += 1;
        }

        double blpos = fmod(encbl.Get() + 0.395, 1) - getbl;
        if (blpos > 0.5)
        {
            blpos -= 1;
        }
        else if (blpos < -0.5)
        {
            blpos += 1;
        }

        double brpos = fmod(encbr.Get() - 0.01, 1) - getbr;
        if (brpos > 0.5)
        {
            brpos -= 1;
        }
        else if (brpos < -0.5)
        {
            brpos += 1;
        }

        rotfl.Set(-flpos * 1.5);
        rotfr.Set(-frpos * 1.5);
        rotbl.Set(-blpos * 1.5);
        rotbr.Set(-brpos * 1.5);

        pidfl.SetReference(0,  rev::spark::SparkBase::ControlType::kVelocity);
        pidfr.SetReference(0,  rev::spark::SparkBase::ControlType::kVelocity);
        pidbl.SetReference(0,  rev::spark::SparkBase::ControlType::kVelocity);
        pidbr.SetReference(0,  rev::spark::SparkBase::ControlType::kVelocity);
    }

    void SetDesiredStates(frc::SwerveModuleState flDesiredState, frc::SwerveModuleState frDesiredState,
        frc::SwerveModuleState blDesiredState, frc::SwerveModuleState brDesiredState)
    {
        frc::Rotation2d flangle = units::radian_t{rotfl.GetEncoder().GetPosition()};
        frc::SwerveModuleState flopt = frc::SwerveModuleState::Optimize(flDesiredState, flangle);
        frc::Rotation2d frangle = units::radian_t{rotfr.GetEncoder().GetPosition()};
        frc::SwerveModuleState fropt = frc::SwerveModuleState::Optimize(frDesiredState, frangle);
        frc::Rotation2d blangle = units::radian_t{rotbl.GetEncoder().GetPosition()};
        frc::SwerveModuleState blopt = frc::SwerveModuleState::Optimize(blDesiredState, blangle);
        frc::Rotation2d brangle = units::radian_t{rotbr.GetEncoder().GetPosition()};
        frc::SwerveModuleState bropt = frc::SwerveModuleState::Optimize(brDesiredState, brangle);

        SetState(flopt, wheelfl, rotfl);
        SetState(fropt, wheelfr, rotfr);
        SetState(blopt, wheelbl, rotbl);
        SetState(bropt, wheelbr, rotbr);
    }

    void SetState(frc::SwerveModuleState optState, rev::spark::SparkMax& driveSpark, rev::spark::SparkMax& steerSpark)
    {
        double angleThr{2*PI/360};
        double deltaAngle = optState.angle.Radians().value() - steerSpark.GetEncoder().GetPosition();
        if ((fabs(optState.speed.value()) < 0.001) && (fabs(deltaAngle) < angleThr)) {
            driveSpark.Set(0);
            steerSpark.Set(0);
        } else {
            driveSpark.GetClosedLoopController().SetReference(optState.speed.value(), rev::spark::SparkBase::ControlType::kVelocity);
            steerSpark.GetClosedLoopController().SetReference(optState.angle.Radians().value(), rev::spark::SparkBase::ControlType::kPosition);
        }
    }

    void DisabledInit() {}

    void DisabledPeriodic() {}
};

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
