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
// #include <frc/interfaces/Gyro.h>
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

    // for encoders, consider changing methods to GetAlternateEncoder with AlternateEncoder::Type::kHallEffect or something if you face an error
    // Setting up steering motors using even CAN bus ID's
    // The constructor parameter is the "analog input channel" to use, corresponding to
    // the RoboRIO AnalogIn pins.
    rev::spark::SparkMax rotfl{2, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController rotpidfl = rotfl.GetClosedLoopController();
    frc::AnalogEncoder encfl{3};
    double lasta = 0;  // last angle? Not used
    rev::spark::SparkMax rotfr{6, rev::spark::SparkLowLevel::MotorType::kBrushless};
    frc::AnalogEncoder encfr{2};
    rev::spark::SparkMax rotbl{4, rev::spark::SparkLowLevel::MotorType::kBrushless};
    frc::AnalogEncoder encbl{0};
    rev::spark::SparkMax rotbr{8, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkRelativeEncoder m_brSteerEnc = rotbr.GetEncoder();
    frc::AnalogEncoder encbr{1};

    // Setting up shooter motors
    rev::spark::SparkMax shoot_top{20, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController pidshoot_top = shoot_top.GetClosedLoopController();
    rev::spark::SparkMax shoot_bottom{21, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController pidshoot_bottom = shoot_bottom.GetClosedLoopController();

    // Setting up intake motors
    rev::spark::SparkMax intake{30, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController pidintake = intake.GetClosedLoopController();

/*
    rev::spark::SparkMax intakedeploy{24, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController pidintakedeploy = intakedeploy.GetClosedLoopController();

    // Setting up hang motor
    rev::spark::SparkMax hangdrive{25, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkClosedLoopController pidhangdrive = hangdrive.GetClosedLoopController();

    // Setting up brushed motor
//    rev::spark::SparkMax brushedmtr{9, rev::spark::SparkLowLevel::MotorType::kBrushed};
//    frc::Spark brushedmtr{9};

    // Setting up servo motor
//    frc::Spark spark{0};
//    bool servoRaised = false;

*/
    int abuttoncount = 0;
    double d = 180;
    double speedfactor = 2000;
    double getto = 3000;
    double changing = false;
    double cfac = 0;

    frc::Timer time;
    bool fieldoriented = false;  // Not used

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

    void RobotInit()
    {
        // wheelfl.SetInverted(true);
        // wheelbr.SetInverted(true);
        // wheelbl.SetInverted(true);
        // wheelfr.SetInverted(true);

        // This section, taken from the REVLib 2025 documentation, uses C++ "method chaining."
        // This is a compact notation whereby multiple calls on a single object can be combined,
        // so long as the methods return a reference to the object itself (as these methods do).
        // This is why all but the final method call in such chained calls omit the semicolon
        // at the end of their respective lines.
        driveConfig
            .Inverted(true)
            .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
        driveConfig.encoder
            .PositionConversionFactor(2 * PI * 2.0 * 0.0254)
            .VelocityConversionFactor(2 * PI * 2.0 * 0.0254 / 60);
//        driveConfig.encoder
//            .PositionConversionFactor(1)
//            .VelocityConversionFactor(1);
        driveConfig.closedLoop
            .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
            .Pid(0.0001, 0.000001, 0.00000001)
            .IZone(4000);

        steerConfig
            .Inverted(true)
            .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
        steerConfig.encoder
            .PositionConversionFactor(2 * PI)
            .VelocityConversionFactor(2 * PI / 60);
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

/*
        pidfl.SetP(0.0001);
        pidfl.SetI(.000001);
        pidfl.SetD(0.00000001);
        pidfl.SetIZone(4000);

        pidfr.SetP(0.0001);
        pidfr.SetI(.000001);
        pidfr.SetD(0.00000001);
        pidfr.SetIZone(4000);

        pidbl.SetP(0.0001);
        pidbl.SetI(.000001);
        pidbl.SetD(0.00000001);
        pidbl.SetIZone(4000);

        pidbr.SetP(0.0001);
        pidbr.SetI(.000001);
        pidbr.SetD(0.00000001);
        pidbr.SetIZone(4000);
*/

/*
        shoot_top.SetInverted(false);
        pidshoot_top.SetP(0.0001);
        pidshoot_top.SetI(.000001);
        pidshoot_top.SetD(0.00000001);
        pidshoot_top.SetIZone(4000);

        shoot_bottom.SetInverted(true);
        pidshoot_bottom.SetP(0.0001);
        pidshoot_bottom.SetI(.000001);
        pidshoot_bottom.SetD(0.00000001);
        pidshoot_bottom.SetIZone(4000);

        intake.SetInverted(false);
        pidintake.SetP(0.0001);
        pidintake.SetI(.000001);
        pidintake.SetD(0.00000001);
        pidintake.SetIZone(4000);
*/

//        spark.Set(0);

        ahrs->Reset();
        ahrs->ResetDisplacement();
        ahrs->SetAngleAdjustment(0);
    }

    void RobotPeriodic() {}

    void AutonomousInit()
    {
        time.Start();
    }

    void TeleopInit()
    {
        time.Stop();
        time.Reset();
        speedfactor = 2000;
    }
/*
    void AutoDrive(double x, double y, double rotate) {

        try
        {
            d = 360-ahrs->GetAngle();
            frc::SmartDashboard::PutString("connection", "connected");
        } catch (int degree) {
            frc::SmartDashboard::PutString("connection", "lost");
        }
        units::degree_t degr {d};
        frc::Rotation2d rot2d {degr};

        units::radians_per_second_t rad {rotate};
        units::meters_per_second_t speedy {y};
        units::meters_per_second_t speedx {x};
        frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(speedy, speedx, rad, rot2d);

        auto [fl, fr, bl, br] = kinematics.ToSwerveModuleStates(speeds);

        // move swerve motors to angles here
        double getfl = fl.angle.Radians().value()/2.0/PI;
        double getfr = fr.angle.Radians().value()/2.0/PI;
        double getbl = bl.angle.Radians().value()/2.0/PI;
        double getbr = br.angle.Radians().value()/2.0/PI;

        double frole = fl.speed.value();
        double frori = fr.speed.value();
        double bale = bl.speed.value();
        double bari = br.speed.value();

        double flpos = encfl.GetAbsolutePosition()-getfl;

        if (flpos > 0.5) {
            flpos -= 1;
        } else if (flpos < -0.5) {
            flpos+=1;
        }

        double frpos = fmod(encfr.GetAbsolutePosition()+.64, 1)-getfr;
        if (frpos > 0.5) {
            frpos -= 1;
        } else if (frpos < -0.5) {
            frpos+=1;
        }

        double blpos = fmod(encbl.GetAbsolutePosition()+.4, 1)-getbl;
        if (blpos > 0.5) {
            blpos -= 1;
        } else if (blpos < -0.5) {
            blpos+=1;
        }

        double brpos = encbr.GetAbsolutePosition()-getbr;
        if (brpos > 0.5) {
            brpos -= 1;
        } else if (brpos < -0.5) {
            brpos+=1;
        }

        rotfl.Set(-flpos*1.5);
        rotfr.Set(frpos*1.5);
        rotbl.Set(-blpos*1.5);
        rotbr.Set(-brpos*1.5);

        if (brpos < .05 && blpos < .05 && flpos < .05 && frpos < .05 && frpos > -.05 && brpos > -.05 && blpos > -.05 && flpos > -.05) {
            pidfl.SetReference(frole*speedfactor, rev::ControlType::kVelocity);
            pidfr.SetReference(frori*speedfactor, rev::ControlType::kVelocity);
            pidbl.SetReference(bale*speedfactor, rev::ControlType::kVelocity);
            pidbr.SetReference(bari*speedfactor, rev::ControlType::kVelocity);
        } else {
            pidfl.SetReference(0, rev::ControlType::kVelocity);
            pidfr.SetReference(0, rev::ControlType::kVelocity);
            pidbl.SetReference(0, rev::ControlType::kVelocity);
            pidbr.SetReference(0, rev::ControlType::kVelocity);
        }
    }
*/
    void AutonomousPeriodic()
    {
        if (time.Get() <= 5_s)
        {
            // spin shooter
            pidshoot_top.SetReference(10000,  rev::spark::SparkBase::ControlType::kVelocity);
            pidshoot_top.SetIAccum(0);
            pidshoot_bottom.SetReference(10000,  rev::spark::SparkBase::ControlType::kVelocity);
            pidshoot_bottom.SetIAccum(0);
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
            // stop shooter
            pidshoot_top.SetReference(0,  rev::spark::SparkBase::ControlType::kVelocity);
            pidshoot_top.SetIAccum(0);
            pidshoot_bottom.SetReference(0,  rev::spark::SparkBase::ControlType::kVelocity);
            pidshoot_bottom.SetIAccum(0);

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

    void Drive0(double x, double y, double rotate)
    {
        // if (x != 0 && y != 0 && rotate != 0) {
        try
        {
            d = 360 - ahrs->GetAngle();
            frc::SmartDashboard::PutString("connection", "connected");
        }
        catch (int degree)
        {
            frc::SmartDashboard::PutString("connection", "lost");
        }
        units::degree_t degr{d};
        frc::Rotation2d rot2d{degr};

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
        frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(limitx.Calculate(speedy), limity.Calculate(speedx), rad * 1.2, rot2d);

        auto [fl, fr, bl, br] = kinematics.ToSwerveModuleStates(speeds);

        // move swerve motors to angles here
        double getfl = fl.angle.Radians().value() / 2.0 / PI;
        double getfr = fr.angle.Radians().value() / 2.0 / PI;
        double getbl = bl.angle.Radians().value() / 2.0 / PI;
        double getbr = br.angle.Radians().value() / 2.0 / PI;

        double frole = fl.speed.value();
        double frori = fr.speed.value();
        double bale = bl.speed.value();
        double bari = br.speed.value();

        double flpos = encfl.Get() - getfl;

        if (flpos > 0.5)
        {
            flpos -= 1;
        }
        else if (flpos < -0.5)
        {
            flpos += 1;
        }

        double frpos = fmod(encfr.Get() + .64, 1) - getfr;
        if (frpos > 0.5)
        {
            frpos -= 1;
        }
        else if (frpos < -0.5)
        {
            frpos += 1;
        }
        double blpos = fmod(encbl.Get() + .4, 1) - getbl;
        if (blpos > 0.5)
        {
            blpos -= 1;
        }
        else if (blpos < -0.5)
        {
            blpos += 1;
        }

        double brpos = encbr.Get() - getbr;
        if (brpos > 0.5)
        {
            brpos -= 1;
        }
        else if (brpos < -0.5)
        {
            brpos += 1;
        }

        rotfl.Set(-flpos * 1.5);
        rotfr.Set(frpos * 1.5);
        rotbl.Set(-blpos * 1.5);
        rotbr.Set(-brpos * 1.5);

        pidfl.SetReference(frole * speedfactor, rev::spark::SparkBase::ControlType::kVelocity);

        pidfr.SetReference(frori * speedfactor, rev::spark::SparkBase::ControlType::kVelocity);

        pidbl.SetReference(bale * speedfactor, rev::spark::SparkBase::ControlType::kVelocity);

        pidbr.SetReference(bari * speedfactor, rev::spark::SparkBase::ControlType::kVelocity);
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
        frc::SmartDashboard::PutNumber("AHRS", rot2d.Degrees().value());
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
z
        double frpos = fmod(encfr.Get() + 0.0, 1) - getfr;
        if (frpos > 0.5)
        {
            frpos -= 1;
        }
        else if (frpos < -0.5)
        {
            frpos += 1;
        }
        double blpos = fmod(encbl.Get() + 0.0, 1) - getbl;
        if (blpos > 0.5)
        {
            blpos -= 1;
        }
        else if (blpos < -0.5)
        {
            blpos += 1;
        }

        double brpos = fmod(encbr.Get() + 0.0, 1) - getbr;
        if (brpos > 0.5)
        {
            brpos -= 1;
        }
        else if (brpos < -0.5)
        {
            brpos += 1;
        }

        frc::SmartDashboard::PutNumber("flpos", flpos);
        frc::SmartDashboard::PutNumber("frpos", frpos);
        frc::SmartDashboard::PutNumber("blpos", blpos);
        frc::SmartDashboard::PutNumber("brpos", brpos);

/*
//        return;

        pidfl.SetReference(0, rev::spark::SparkBase::ControlType::kVelocity);
        rotfl.Set(0);
//        rotpidfl.SetReference(10, rev::spark::SparkBase::ControlType::kPosition);  // nominally in rotations, 

///        return;

        pidfl.SetReference(5, rev::spark::SparkBase::ControlType::kVelocity);
        rotfl.Set(0);

        pidbl.SetReference(5, rev::spark::SparkBase::ControlType::kVelocity);
        rotbl.Set(0);

        return;
*/

        // rotfl.Set(-flpos * 1.5);
        rotfl.Set(-flpos * 0.1);
        rotfr.Set(frpos * 0.1);
        rotbl.Set(-blpos * 0.1);
        rotbr.Set(-brpos * 0.1);

        pidfl.SetReference(frole * speedfactor,  rev::spark::SparkBase::ControlType::kVelocity);
        pidfr.SetReference(frori * speedfactor,  rev::spark::SparkBase::ControlType::kVelocity);
        pidbl.SetReference(bale * speedfactor,  rev::spark::SparkBase::ControlType::kVelocity);
        pidbr.SetReference(bari * speedfactor,  rev::spark::SparkBase::ControlType::kVelocity);
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
        frc::SmartDashboard::PutNumber("fl", wheelfl.Get());  // wheelfl.Get() returns the current set speed in [-1,1]
        frc::SmartDashboard::PutNumber("fr", wheelfr.Get());
        frc::SmartDashboard::PutNumber("bl", wheelbl.Get());
        frc::SmartDashboard::PutNumber("br", wheelbr.Get());

        //controller 1
        if ((controller.GetRightTriggerAxis() > 0.1) && !(controller.GetLeftTriggerAxis() > 0.1))
        {
            intake.Set(0.5);
//            pidintake.SetReference(5000,  rev::spark::SparkBase::ControlType::kVelocity);
            pidintake.SetIAccum(0);
        }
        else if (!(controller.GetRightTriggerAxis() > 0.1) && (controller.GetLeftTriggerAxis() > 0.1))
        {
            // Will SetReference to a negative value achieve what we want? Or will we need to switch to .Set() instead?
            intake.Set(-0.5);
//            pidintake.SetReference(-5000,  rev::spark::SparkBase::ControlType::kVelocity);
            pidintake.SetIAccum(0);
        }
        else if (!(controller.GetRightTriggerAxis() > 0.1) && !(controller.GetLeftTriggerAxis() > 0.1))
        {
            intake.Set(0.0);
//            pidintake.SetReference(0,  rev::spark::SparkBase::ControlType::kVelocity);
            pidintake.SetIAccum(0);
        }

        if (controller.GetAButton())
        {
            pidfl.SetIAccum(0);
            pidfr.SetIAccum(0);
            pidbl.SetIAccum(0);
            pidbr.SetIAccum(0);
        }
        else if (controller.GetXButton())
        {
            xstop();
            pidfl.SetIAccum(0);
            pidfr.SetIAccum(0);
            pidbl.SetIAccum(0);
            pidbr.SetIAccum(0);
        }
        else
        {
            if (controller.GetRightBumper())
            {
                if (speedfactor < 6000)
                {
                    speedfactor += 20;
                }
            }
            else if (controller.GetLeftBumper())
            {
                if (speedfactor > 1000)
                {
                    speedfactor -= 20;
                }
            }
            double drift = 0.15;
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

            if (controller2.GetRightTriggerAxis() > 0.1)
            {
                /*
                shoot_speed += add_value;
                if (shoot_speed < 0) shoot_speed = 0;
                if (shoot_speed > speed_cap) shoot_speed = speed_cap;
                */

                shoot_top.SetInverted(false);
                pidshoot_top.SetReference(10000,  rev::spark::SparkBase::ControlType::kVelocity);
                pidshoot_top.SetIAccum(0);
                shoot_bottom.SetInverted(true);
                pidshoot_bottom.SetReference(10000,  rev::spark::SparkBase::ControlType::kVelocity);
                pidshoot_bottom.SetIAccum(0);
            }

            else
            {
                pidshoot_top.SetReference(0,  rev::spark::SparkBase::ControlType::kVelocity);
                pidshoot_top.SetIAccum(0);
                pidshoot_bottom.SetReference(0,  rev::spark::SparkBase::ControlType::kVelocity);
                pidshoot_bottom.SetIAccum(0);
            }

            //sleep(20);
        //}
/*
        if (controller2.GetAButton() && !controller2.GetBButton())
        {
            //intake extend
            intakedeploy.Set(0.1);
//            intakedeploy.SetInverted(false);
//            pidintakedeploy.SetReference(2000,  rev::spark::SparkBase::ControlType::kVelocity);
            pidintakedeploy.SetIAccum(0);
        }
        else if (!controller2.GetAButton() && controller2.GetBButton())
        {
            //intake retract
            intakedeploy.Set(-0.1);
//            pidintakedeploy.SetReference(0,  rev::spark::SparkBase::ControlType::kVelocity);
            pidintakedeploy.SetIAccum(0);
        }
        else if (!controller2.GetAButton() && !controller2.GetBButton())
        {
            intakedeploy.Set(0.0);
            pidintakedeploy.SetIAccum(0);
        }
*/
/*        if (controller2.GetYButton())
        {
            intakedeploy.SetInverted(true);
            pidintakedeploy.SetReference(2000,  rev::spark::SparkBase::ControlType::kVelocity);
            pidintakedeploy.SetIAccum(0);
        }
        else if (!controller2.GetYButton())
        {
            pidintakedeploy.SetReference(0,  rev::spark::SparkBase::ControlType::kVelocity);
            pidintakedeploy.SetIAccum(0);
        }
*/
/*
//used to be else if
        if (controller2.GetLeftBumper())
        {
            hangdrive.Set(0.2);
//           hangdrive.SetInverted(true);
//            pidhangdrive.SetReference(2000,  rev::spark::SparkBase::ControlType::kVelocity);
            pidhangdrive.SetIAccum(0);
        }
//        else if (!controller2.GetLeftBumper())
        else
        {
            hangdrive.Set(0.0);
//            pidhangdrive.SetReference(0,  rev::spark::SparkBase::ControlType::kVelocity);
            pidhangdrive.SetIAccum(0);
        }
*/
/*        if (controller2.GetRightBumper() && !controller2.GetLeftBumper())
        {
            brushedmtr.Set(-0.2);
        }
        else if (!controller2.GetRightBumper() && controller2.GetLeftBumper())
        {
            brushedmtr.Set(0.2);
        }
        else if (!controller2.GetRightBumper() && !controller2.GetLeftBumper())
        {
            brushedmtr.Set(0.0);
        }
*/
/*
        int pov = controller2.GetPOV();
        if (pov >= 0)
        {
            int hangdir = (pov - 90) / (-90);
            hangdrive.Set(hangdir * 0.5);
        } else {
            hangdrive.Set(0.0);
        }
*/
/*
        if (controller2.GetRightBumper())
        {
            hangdrive.Set(-0.2);
//            hangdrive.SetInverted(false);
//            pidhangdrive.SetReference(2000,  rev::spark::SparkBase::ControlType::kVelocity);
            pidhangdrive.SetIAccum(0);
        }
        else if (!controller2.GetRightBumper())
        {
            hangdrive.Set(0.0);
//            pidhangdrive.SetReference(0,  rev::spark::SparkBase::ControlType::kVelocity);
            pidhangdrive.SetIAccum(0);
        }
*/
/*      else if (controller2.GetAButton())
        {
            if (servoRaised)
            {
                spark.Set(0);
                servoRaised = false;
            } else {
                spark.Set(1);
                servoRaised = true;
            }
        }
*/
    }

    void xstop()
    {
        // move swerve motors to angles here
        double getfl = .125;
        double getfr = .825;
        double getbl = .825;
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
        double flpos = encfl.Get() - getfl;
        if (flpos > 0.5)
        {
            flpos -= 1;
        }
        else if (flpos < -0.5)
        {
            flpos += 1;
        }

        double frpos = fmod(encfr.Get() + .14, 1) - getfr;
        if (frpos > 0.5)
        {
            frpos -= 1;
        }
        else if (frpos < -0.5)
        {
            frpos += 1;
        }

        double blpos = fmod(encbl.Get() + .4, 1) - getbl;
        if (blpos > 0.5)
        {
            blpos -= 1;
        }
        else if (blpos < -0.5)
        {
            blpos += 1;
        }

        double brpos = encbr.Get() - getbr;
        if (brpos > 0.5)
        {
            brpos -= 1;
        }
        else if (brpos < -0.5)
        {
            brpos += 1;
        }

        rotfl.Set(-flpos * 1.5);
        rotfr.Set(frpos * 1.5);
        rotbl.Set(-blpos * 1.5);
        rotbr.Set(-brpos * 1.5);

        pidfl.SetReference(0,  rev::spark::SparkBase::ControlType::kVelocity);

        pidfr.SetReference(0,  rev::spark::SparkBase::ControlType::kVelocity);

        pidbl.SetReference(0,  rev::spark::SparkBase::ControlType::kVelocity);

        pidbr.SetReference(0,  rev::spark::SparkBase::ControlType::kVelocity);
    }

    void xstop2()
    {

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
