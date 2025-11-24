package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    /*  NOTICE --- The final line written under these static constants has to end in ";" and be the only one 'ended'.
    The order of testing is as follows; V V V
    1. localization(DriveEncoderConstants) !!!Done!!!
    2. Automatic Tuners, AKA tuning opmode,(MecanumConstants, then FollowerConstants)
    Start referring to Panels(192.168.43.1:8001) for values
    3. Dual PID System (FollowerConstants) if we want to use it
    4. Translational (FollowerConstants)
    5. Heading (FollowerConstants)
    6. Drive (FollowerConstants)
    7. Centripetal (FollowerConstants)
    Done tuning! Go to tests!
    */
    // pounds-to-kg = 0.453592 * (robot weight in pounds)
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.981);
            /*
            .forwardZeroPowerAcceleration(deceleration on driver hub after tuning opmode(Forward Zero Power Acceleration))
            .lateralZeroPowerAcceleration(deceleration on driver hub after tuning opmode(Lateral Zero Power Acceleration))
            */
            /*  Dual PID System (after Tuning OpMode) if we want to use it
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
                then start tuning these (vids on the PP website)
            */
            /*  Translational (set P, I, D, and F)
            .translationalPIDFCoefficients(new PIDFCoefficients(P, I, D, F))
            if using dual PIDF VVV (could be diff P,I,D,and F values)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(P,I,D,F))
            */
            /*  Heading (set NEW P, I, D, and F)
            .headingPIDFCoefficients(new PIDFCoefficients(P, I, D, F))
            if using dual PIDF VVV (could be diff P,I,D,and F values)
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(P,I,D,F))
            */
            /*  Drive (set NEW P, I, D, T, and F)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(P, I, D, T, F))
            if using dual PIDTF VVV (could be diff P, I, D, T, and F values)
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(P, I, D, T, F))
            */
            /* Centripetal
            .centripetalScaling(value on Panels after CentripetalTuner)
            */

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.6)
            .rightFrontMotorName("RF")
            .rightRearMotorName("RB")
            .leftRearMotorName("LB")
            .leftFrontMotorName("LF")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            /*  Tuning Opmode (Automatic Tuners)
            .xVelocity() // velocity on driver hub after tuning opmode(Forward Velocity Tuner)
            .yVelocity(); // velocity on driver hub after tuning opmode(Lateral Velocity Tuner)
*/ ;
    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .rightFrontMotorName("RF")
            .rightRearMotorName("RB")
            .leftRearMotorName("LB")
            .leftFrontMotorName("LF")
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .rightFrontEncoderDirection(Encoder.REVERSE)
            .rightRearEncoderDirection(Encoder.REVERSE)
            .robotWidth(14)
            .robotLength(17)
            // Tuning Localizer
            .forwardTicksToInches(-0.02060969254715356) // multiplier on driver hub after localization(forwardtuner) Distance(from 72) = -2257.0012643410037     Multiplier = -0.02060969254715356
            .strafeTicksToInches(-0.033693672508588346) // multiplier on driver hub after localization(lateraltuner) Distance(from 72) = -1352.599826066876     Multiplier = -0.033693672508588346
            .turnTicksToInches(0.021291948461146404); // multiplier on driver hub after localization(lateraltuner) turned countercw

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .driveEncoderLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}