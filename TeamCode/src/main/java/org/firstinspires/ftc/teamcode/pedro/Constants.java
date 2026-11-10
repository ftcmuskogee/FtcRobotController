package org.firstinspires.ftc.teamcode.pedro;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
        public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
            c.frontLeftName.set("FL");
            c.frontRightName.set("FR");
            c.backLeftName.set("BL");
            c.backRightName.set("BR");
            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        }
        );
        public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-0.0);
        c.yPodOffset.set(-0.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    }
    );
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.23618267712483912);
                Controller secondaryTranslationalForward = Controller.proportional(0.08726318828026544);
                Controller primaryTranslationalLateral = Controller.proportional(0.356584685315434);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1317485131057794);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011030747286216207));
                c.brake.set(Controller.proportionalFeedforward(0.009376135193283777));

                c.headingFeedback.set(Controller.proportional(3.4719290128338702));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0733297266032531, 0.0026512522483335983));

                c.linearBrakeCoefficients.set(Matrix.diag(0.19334956745904114, 0.07300846227839626));
                c.quadraticBrakeCoefficients.set(Matrix.diag(-5.589919499434834E-4, 0.0016436469663177293));

                c.maxAchievableForwardVelocity.set(87.82126243915056);
                c.maxAchievableStrafeVelocity.set(9.193313408331415);
                c.naturalForwardDeceleration.set(33.088430859431);
                c.naturalStrafeDeceleration.set(67.63798021467655);
            }
    );

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}


