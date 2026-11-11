package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "MainCode", group = "Autonomous")
public class mainCode extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose autoStart = poseFactory.of(56, 9, 90);
    private final Pose setShoot1 = poseFactory.of(38, 14, 65);
    private final Pose setShoot1Control1 = poseFactory.of(51, 16, 0);
    private final Pose setToIntakeStart = poseFactory.of(38, 14, 70);
    private final Pose setToIntake = poseFactory.of(9, 25, -90);
    private final Pose intakeStart = poseFactory.of(9, 25, -90);
    private final Pose intake = poseFactory.of(9, 19, -90);
    private final Pose setShoot2Start = poseFactory.of(9, 19, -90);
    private final Pose setShoot2 = poseFactory.of(28, 130, -65);
    private final Pose setShoot2Control1 = poseFactory.of(24, 30, 0);
    private final Pose setShoot2Control2 = poseFactory.of(20, 60, 0);
    private final Pose setShoot2Control3 = poseFactory.of(20, 100, 0);
    private final Pose setParkStart = poseFactory.of(28, 130, -65);
    private final Pose setPark = poseFactory.of(10, 130, -90);
    private final Pose park = poseFactory.of(10, 120, -90);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, setShoot1()),
                follow(follower, setToIntake()),
                follow(follower, intake()),
                follow(follower, setShoot2()),
                follow(follower, setPark()),
                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(autoStart);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path setShoot1() {
        return Paths.curve(autoStart, setShoot1Control1, setShoot1).linear(autoStart, setShoot1);
    }

    public Path setToIntake() {
        return Paths.line(setToIntakeStart, setToIntake).linear(setToIntakeStart, setToIntake);
    }

    public Path intake() {
        return Paths.line(intakeStart, intake).linear(intakeStart, intake);
    }

    public Path setShoot2() {
        return Paths.curve(setShoot2Start, setShoot2Control1, setShoot2Control2, setShoot2Control3, setShoot2).linear(setShoot2Start, setShoot2);
    }

    public Path setPark() {
        return Paths.line(setParkStart, setPark).linear(setParkStart, setPark);
    }

    public Path park() {
        return Paths.line(setPark, park).constant(park);
    }
}
