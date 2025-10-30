package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Set", group="Servo")
public class ServoPositions extends LinearOpMode {
    private Servo shooterReloader = null;

    @Override
    public void runOpMode() throws InterruptedException {

        shooterReloader = hardwareMap.get(Servo.class, "SR");

        shooterReloader.setDirection(Servo.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // show current position when CIRCLE is pressed
            if (gamepad1.aWasPressed()) {
                telemetry.addData("0 value is...", shooterReloader.getPosition());
                telemetry.update();
            }

            // show current position when SQUARE is pressed
            if (gamepad1.bWasPressed()) {
                telemetry.addData("1 value is...", shooterReloader.getPosition());
                telemetry.update();
            }
        }
    }

}
