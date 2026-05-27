package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Back BLUE Main", group = "PP Autonomous", preselectTeleOp = "MecanumTeleOp")
@Configurable
public class BlueBACKPP extends OpMode {

    // ================= TELEMETRY =================
    private TelemetryManager panelsTelemetry;

    // ================= PEDRO =================
    private Follower follower;
    private Paths paths;

    // ================= HARDWARE =================
    private DcMotor shooterMotor1;
    private DcMotor shooterMotor2;
    private DcMotor intakeMotor;
    private Servo Gate;
    private Servo Hood;

    // ================= STATE =================
    private AutoState state;
    private AutoState lastState;
    private long stateStartTime;
    private boolean pathStarted = false;
    public int stopEarly = 0;
    public double shootPow = 1;

    private enum AutoState {
        SHOOT,
        Off,
        DONE
    }

    // ================= FLEXIBLE START POSE =================
    public Pose startPose = new Pose(61, 8, Math.toRadians(140));

    // ================= INIT =================
    @Override
    public void init() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        shooterMotor1 = hardwareMap.get(DcMotor.class, "S1");
        shooterMotor2 = hardwareMap.get(DcMotor.class, "S2");
        intakeMotor = hardwareMap.get(DcMotor.class, "NTK");
        Gate = hardwareMap.get(Servo.class,"Gate");
        Hood = hardwareMap.get(Servo.class, "Hood");

        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startPose);

        paths = new Paths(follower);

        Gate.setPosition(1);
        Hood.setPosition(1);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    // ================= START =================
    public void start() {
        state = AutoState.SHOOT;
        lastState = null;
        stateStartTime = System.currentTimeMillis();
    }

    // ================= LOOP =================
    @Override
    public void loop() {

        follower.update();
        updateStateTimer();
        updateAuto();

        panelsTelemetry.addData("State - ", state);
        panelsTelemetry.addData("X - ", Math.round(follower.getPose().getX()));
        panelsTelemetry.addData("Y - ", Math.round(follower.getPose().getY()));
        panelsTelemetry.addData("Heading - ", Math.round(Math.toDegrees(follower.getPose().getHeading())));
        panelsTelemetry.addData("Shooter Power - ", Math.round(shootPow));
        panelsTelemetry.addData("Gate Status - ", (Gate.getPosition() == 0 ? "OPEN" : "CLOSED"));
        panelsTelemetry.update(telemetry);
    }

    // ================= STATE TIMER =================
    private void updateStateTimer() {
        if (state != lastState) {
            stateStartTime = System.currentTimeMillis();
            lastState = state;
            pathStarted = false;
        }
    }

    private long elapsed() {
        return System.currentTimeMillis() - stateStartTime;
    }

    // ================= AUTO =================
    private void updateAuto() {

        switch (state) {

            case SHOOT: //                                                                         SHOOT 1
                if (elapsed() >= 2000) Gate.setPosition(0); else Gate.setPosition(.65);
                shooterMotor1.setPower(shootPow);
                shooterMotor2.setPower(shootPow);
                if (!follower.isBusy() && (elapsed() >= 2850 && elapsed() <= 4250)) {
                    intakeMotor.setPower(0.75);
                } else if (elapsed() > 4250) {
                    stopMotors();
                    transitionTo(AutoState.Off);
                }
                break;

            case Off: //                                                                            OFF
                Gate.setPosition(.65);
                stopEarly = 3;
                shootPow = 0.25;
                followOnce(paths.off(), AutoState.DONE);
                break;

            case DONE: //                                                                           DONE
                Gate.setPosition(.65);
                stop();
                break;
        }
    }

    // ================= HELPERS =================
    private void stopMotors() {
        shootPow = 0.8;
        intakeMotor.setPower(0);
    }

    private void transitionTo(AutoState next) {
        state = next;
    }

    private void startFollow(PathChain path) {
        if (!pathStarted) {
            follower.followPath(path);
            pathStarted = true;
        }
    }
    private void followOnce(PathChain path, AutoState next) {
        if (!pathStarted) {
            follower.followPath(path);
            pathStarted = true;
        }

        double inchesOff =
                stopEarly == 0 ? 2 :
                        stopEarly == 1 ? 1 :
                                stopEarly == 2 ? 0.5 :
                                        3;

        if (follower.isRobotStuck()) {
            transitionTo(next);
        } else if (follower.isBusy() && follower.getDistanceRemaining() <= inchesOff) {
            transitionTo(next);
        } else if (!follower.isBusy()) {
            transitionTo(next);
        }
    }

    // ================= PATHS =================
    public static class Paths {

        private final Follower follower;

        public Paths(Follower follower) {
            this.follower = follower;
        }

        private Pose pathStartPose() {
            return follower.getPose();
        }

        public PathChain off() {
            return follower.pathBuilder()
                    .addPath(new BezierLine(pathStartPose(), new Pose(50, 10)))
                    .setConstantHeadingInterpolation(pathStartPose().getHeading())
                    .build();
        }
    }
}