package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Red Auto - Optional", group = "Red")
public class RedAutoOffBack extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    private DcMotor shooterMotor = null;
    private Servo servo = null;

    private ElapsedTime runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6; // For directions from the left joystick.
    static final double     TURN_SPEED    = 0.5; // For rotations from the right joystick.

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("LF");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("LB");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("RF");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("RB");
        DcMotor shooterMotor = hardwareMap.get(DcMotor.class, "SM");
        Servo servo = hardwareMap.get(Servo.class, "servo");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        servo.setPosition(0.04);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        //sleep(5000); also works, but this is more clear
        while (opModeIsActive() && (runtime.seconds() < 5.0)) {   // 5000 milliseconds
            telemetry.addData("Path", "Idle: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Move to launch line
        frontLeftMotor.setPower(FORWARD_SPEED);
        frontRightMotor.setPower(FORWARD_SPEED);
        backLeftMotor.setPower(FORWARD_SPEED);
        backRightMotor.setPower(FORWARD_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.25)) {   // 1250 milliseconds
            telemetry.addData("Path", "Moving to Launch Position: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();

        }
        // Stop
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(100);

        //on launch line
        frontLeftMotor.setPower(TURN_SPEED);
        frontRightMotor.setPower(-TURN_SPEED);
        backLeftMotor.setPower(TURN_SPEED);
        backRightMotor.setPower(-TURN_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {   // 500 milliseconds
            telemetry.addData("Path", "Turning to goal: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Stop
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(100);

        shooterMotor.setPower(1);
        sleep(500);
        for (int i = 1; i < 3 ; i++) {
            servo.setPosition(0);
            sleep(250);
            servo.setPosition(0.04);
        }
        shooterMotor.setPower(0);

        resetRuntime();
        while (opModeIsActive() && (runtime.seconds()<0.5)) {
            telemetry.addData("Path", "Shooting...: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        frontLeftMotor.setPower(FORWARD_SPEED);
        frontRightMotor.setPower(-FORWARD_SPEED);
        backLeftMotor.setPower(-FORWARD_SPEED);
        backRightMotor.setPower(FORWARD_SPEED);

        while (opModeIsActive() && (runtime.seconds() < 0.5)) {   // 500 milliseconds
            telemetry.addData("Path", "Moving Off of Launch Line: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Stop
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(100);

        // current code = 12 second auto


        


    }

}

