package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

@Disabled
public class Constants {

    /*  NOTICE --- The final line written under these static constants has to end in ";" and be the only one 'ended'.
    The order of testing is as follows; V V V
    1. localization(DriveEncoderConstants)
    2. Automatic Tuners, AKA tuning opmode,(MecanumConstants, then FollowerConstants)
    Start referring to Panels(192.168.43.1:8001) for values
    3. Dual PID System (FollowerConstants) if we want to use it !!!Not Now!!!
    4. Translational (FollowerConstants)
    5. Heading (FollowerConstants)
    6. Drive (FollowerConstants)
    7. Centripetal (FollowerConstants)
    Done tuning! Go to tests!
    */
    // pounds-to-kg = 0.453592 * (robot weight in pounds)
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(4.4452016)
            // Tuning OpMode (Automatic Tuner)
            //.forwardZeroPowerAcceleration() // deceleration on driver hub after tuning opmode(Forward Zero Power Acceleration)
            //.lateralZeroPowerAcceleration() // deceleration on driver hub after tuning opmode(Lateral Zero Power Acceleration)
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
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.6)
            .rightFrontMotorName("RF")
            .rightRearMotorName("RB")
            .leftRearMotorName("LB")
            .leftFrontMotorName("LF")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            //  Tuning Opmode (Automatic Tuners)
            //.xVelocity() // velocity on driver hub after tuning opmode(Forward Velocity Tuner)
            //.yVelocity() // velocity on driver hub after tuning opmode(Lateral Velocity Tuner)
            ;
    //public static TwoDeadWheelLocalizer localizerConstants = new TwoDeadWheelLocalizer();


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 3000, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                //.TwoDeadWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}