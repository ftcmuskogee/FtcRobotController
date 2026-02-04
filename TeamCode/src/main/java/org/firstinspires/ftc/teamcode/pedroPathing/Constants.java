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
    1. localization(PinpointConstants)
    2. Automatic Tuners, AKA tuning opmode,(MecanumConstants, then FollowerConstants)
    Start referring to Panels(192.168.43.1:8001) for values
    ?. Dual PID System (FollowerConstants) if we want to use it !!!Not Now!!!
    3. Translational PIDF (FollowerConstants)
    4. Heading PIDF (FollowerConstants)
    5. Drive PIDTF (FollowerConstants)
    6. Centripetal Scale (FollowerConstants)
    Done tuning! Go do tests!
    */
    // pounds-to-kg = 0.453592 * (robot weight in pounds)
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.79759519)
         // Tuning OpMode (Automatic Tuner)
            .forwardZeroPowerAcceleration(-38.15687812211297) // deceleration on driver hub after tuning opmode(Forward Zero Power Acceleration)
            .lateralZeroPowerAcceleration(-55.984695759171636) // deceleration on driver hub after tuning opmode(Lateral Zero Power Acceleration)
            // Dual PID System (after Tuning OpMode) if we want to use it, set following to "true"
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            // Translational (set P, I, D, and F)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.007, 0, 0.001, 0.02))
            // if using dual PIDF VVV (could be diff P,I,D,and F values)
            //.secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(P,I,D,F))
            //  Heading (set NEW P, I, D, and F)
            .headingPIDFCoefficients(new PIDFCoefficients(0.4, 0, 0.001, 0.01))
            //if using dual PIDF VVV (could be diff P,I,D,and F values)
            //.secondaryHeadingPIDFCoefficients(new PIDFCoefficients(P,I,D,F))
            //  Drive (set NEW P, I, D, T, and F)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, 0.00007, 0.5, 0.0045))
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
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            // Tuning Opmode (Automatic Tuners)
            .xVelocity(50.53694837675321) // velocity on driver hub after tuning opmode(Forward Velocity Tuner)
           .yVelocity(33.56449565737266) // velocity on driver hub after tuning opmode(Lateral Velocity Tuner)
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
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
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