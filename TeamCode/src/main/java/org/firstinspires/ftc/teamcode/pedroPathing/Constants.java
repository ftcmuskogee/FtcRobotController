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
     tuning! Go do tests!
    */
    // pounds-to-kg = 0.453592 * (robot weight in pounds)
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.2511792)
         // Tuning OpMode (Automatic Tuner)
            .forwardZeroPowerAcceleration(-26.6869128836423) // deceleration on driver hub after tuning opmode(Forward Zero Power Acceleration)
            .lateralZeroPowerAcceleration(-71.08311597466205) // deceleration on driver hub after tuning opmode(Lateral Zero Power Acceleration)
         // Dual PID System (after Tuning OpMode) if we want to use it, set following to "true"
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
         // Translational (set P, I, D, and F)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.0225, 0, 0.0005, 0.02675))
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
            .maxPower(0.75)
            .rightFrontMotorName("FR")
            .rightRearMotorName("BR")
            .leftRearMotorName("BL")
            .leftFrontMotorName("FL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            // Tuning Opmode (Automatic Tuners)
            .xVelocity(53.85532120832308) // velocity on driver hub after tuning opmode(Forward Velocity Tuner)
           .yVelocity(43.03991747277929) // velocity on driver hub after tuning opmode(Lateral Velocity Tuner)
            ;
    public static PinpointConstants localizerConstants = new PinpointConstants()
            // Forward/Backward is +X/-X
            // Left/Right is +Y/-Y
            .forwardPodY(-(6.85)) // Distance from center (14/2) (y axis) // ||   // On back right
            .strafePodX((8.375)) // Distance from center (17.75/2) (x axis)  // --   // On back left
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("ODO") // Ohh, Dee, Ohh (all capital)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            // !test odo directions! if forward/left decrease, reverse them accordingly
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.75, 1975, 1, 1.001);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build()
                ;
    }
}