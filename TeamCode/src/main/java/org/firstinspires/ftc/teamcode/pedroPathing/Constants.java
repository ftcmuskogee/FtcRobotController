package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    /*  NOTICE --- The final line written under these static constants has to end in ";" and be the only one 'ended'.
    The order of testing is as follows; V V V
    1. localization(PinpointConstants)   !!!DONE!!!
    2. Automatic Tuners, AKA tuning opmode,(MecanumConstants, then FollowerConstants)   !!!DONE!!!
    Start referring to Panels(192.168.43.1:8001) for values
    ?. Dual PID System (FollowerConstants) if we want to use it !!!Not Now!!!
    3. Translational PIDF (FollowerConstants)   !!!DONE!!!
    4. Heading PIDF (FollowerConstants)   !!!DONE!!!
    5. Drive PIDTF (FollowerConstants)   !!!DONE!!!
    6. Centripetal Scale (FollowerConstants)   !!!DONE!!!
    THANK YOU AN!!!
    Done tuning! Go do tests!
    */
    // pounds-to-kg = 0.453592 * (robot weight in pounds)
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.79759519)
         // Tuning OpMode (Automatic Tuner)
            .forwardZeroPowerAcceleration(-38.918110060192994) // deceleration on driver hub after tuning opmode(Forward Zero Power Acceleration)
            .lateralZeroPowerAcceleration(-60.82386942408424) // deceleration on driver hub after tuning opmode(Lateral Zero Power Acceleration)
            // Dual PID System (after Tuning OpMode) if we want to use it, set following to "true"
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            // Translational (set P, I, D, and F)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.02, 0, 0.001, 0.033))
            // if using dual PIDF VVV (could be diff P,I,D,and F values)
            //.secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(P,I,D,F))
            //  Heading (set NEW P, I, D, and F)
            .headingPIDFCoefficients(new PIDFCoefficients(0.6, 0, 0.002, 0.01))
            //if using dual PIDF VVV (could be diff P,I,D,and F values)
            //.secondaryHeadingPIDFCoefficients(new PIDFCoefficients(P,I,D,F))
            //  Drive (set NEW P, I, D, T, and F)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0075, 0, 0.00003, 0.5, 0.0045))
            //if using dual PIDTF VVV (could be diff P, I, D, T, and F values)
            //.secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(P, I, D, T, F))
            .centripetalScaling(0.008)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.6)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)

            // Tuning Opmode (Automatic Tuners)
            .xVelocity(51.601775882750985) // velocity on driver hub after tuning opmode(Forward Velocity Tuner)
           .yVelocity(34.51689135934424) // velocity on driver hub after tuning opmode(Lateral Velocity Tuner)
            ;
    public static PinpointConstants localizerConstants = new PinpointConstants()
            // Forward/Backward is +X/-X
            // Left/Right is +Y/-Y
            .forwardPodY(-(6.5)) // Distance from center (14/2) (y axis) // ||
            .strafePodX((8.5)) // Distance from center (18/2) (x axis)  // --
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("ODO") // Ohh, Dee, Ohh (all capital)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            // !test odo directions! if forward/left decrease, reverse them accordingly
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.75, 2500, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build()
                ;
    }
}