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
    localization(DriveEncoderConstants), tuning opmode(MecanumConstants, then FollowerConstants)
    */
    // pounds2kg = 0.453592 * robot pounds ;
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.981);
            /*
            .forwardZeroPowerAcceleration(deceleration on driver hub after tuning opmode(Forward Zero Power Acceleration))
            .lateralZeroPowerAcceleration(deceleration on driver hub after tuning opmode(Lateral Zero Power Acceleration))
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
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);
            /*  Tuning Opmode (after Localization Tuners VVV Bottom of DriveEncoderConstants VVV)
            .xVelocity(velocity on driver hub after tuning opmode(Forward Velocity Tuner))
            .yVelocity(velocity on driver hub after tuning opmode(Lateral Velocity Tuner))

            */

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
            /*  Tuning localizer
            .forwardTicksToInches(multiplier on driver hub after localization(forwardtuner))
            .strafeTicksToInches(multiplier on driver hub after localization(lateraltuner))
            .turnTicksToInches(multiplier on driver hub after localization(lateraltuner) turned countercw)
                Go test! :)
            */
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .driveEncoderLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}