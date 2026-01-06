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
    1. localization(PinpointConstants) !Done!
    2. Automatic Tuners, AKA tuning opmode,(MecanumConstants, then FollowerConstants) !Done!
    Start referring to Panels(192.168.43.1:8001) for values
    ?. Dual PID System (FollowerConstants) if we want to use it !!!Not Now!!!
    3. Translational PIDF (FollowerConstants) !Done!
    4. Heading PIDF (FollowerConstants) !Done!
    5. Drive PIDTF (FollowerConstants) !Done!
    6. Centripetal Scale (FollowerConstants) !Done!
    Done tuning! Go do tests! !Done!
    */
    // pounds-to-kg = 0.453592 * (robot weight in pounds)
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(4.4452016)
            // Tuning OpMode (Automatic Tuner)
            .forwardZeroPowerAcceleration(-52.57631940261318) // deceleration on driver hub after tuning opmode(Forward Zero Power Acceleration)
            .lateralZeroPowerAcceleration(-63.7450812952701) // deceleration on driver hub after tuning opmode(Lateral Zero Power Acceleration)
            // Dual PID System (after Tuning OpMode) if we want to use it, set following to "true"
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            // Translational (set P, I, D, and F)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.0525, 0, 0.00330, 0.03))
            // if using dual PIDF VVV (could be diff P,I,D,and F values)
            //.secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(P,I,D,F))
            //  Heading (set NEW P, I, D, and F)
            .headingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.001, 0.0225))
            //if using dual PIDF VVV (could be diff P,I,D,and F values)
            //.secondaryHeadingPIDFCoefficients(new PIDFCoefficients(P,I,D,F))
            //  Drive (set NEW P, I, D, T, and F)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, 0.00007, 0.5, 0.0045))
            /*if using dual PIDTF VVV (could be diff P, I, D, T, and F values)
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(P, I, D, T, F))
            */
            .centripetalScaling(0.008)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.45)
            .rightFrontMotorName("RF")
            .rightRearMotorName("RB")
            .leftRearMotorName("LB")
            .leftFrontMotorName("LF")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            //  Tuning Opmode (Automatic Tuners)
            .xVelocity(50.48624546321359) // velocity on driver hub after tuning opmode(Forward Velocity Tuner)
            .yVelocity(39.06283064714567) // velocity on driver hub after tuning opmode(Lateral Velocity Tuner)
            ;
    public static PinpointConstants localizerConstants = new PinpointConstants()
            // Forward/Backward is +X/-X
            // Left/Right is +Y/-Y
            .forwardPodY(-(14.8 / 2)) // width divided by 2 (wheel is centered left/right)  // Distance from center (y axis) // ||
            .strafePodX(-(17.2 / 2)) // length divided by 2 (wheel is centered up/down)  // Distance from center (x axis)    // --
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("ODO") // Ohh, Dee, Ohh (all capital)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            // !test odo directions! if forward/left decrease, reverse them accordingly
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            ;


    public static PathConstraints pathConstraints = new PathConstraints(0.97, 3000, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build()
                ;
    }
}