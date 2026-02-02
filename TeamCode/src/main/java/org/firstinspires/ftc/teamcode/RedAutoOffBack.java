package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Red Auto - Optional", group = "Red", preselectTeleOp = "MecanumTeleop")
public class RedAutoOffBack extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    private DcMotor shooterMotor = null;


    private ElapsedTime runtime = new ElapsedTime();


    static final double FORWARD_SPEED = 0.6; // For directions from the left joystick.
    static final double TURN_SPEED = 0.5; // For rotations from the right joystick.

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("LF");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("LB");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("RF");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("RB");
        DcMotor shooterMotor = hardwareMap.get(DcMotor.class, "SM");
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "NTK");
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // If no input, the robot won't drift
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        runtime.reset();
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
        while (opModeIsActive() && (runtime.seconds() < 1.7)) {   // 1700 milliseconds
            telemetry.addData("Path", "Moving to Launch Position: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Stop
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        sleep(100);

        //on launch line
        frontLeftMotor.setPower(TURN_SPEED);
        frontRightMotor.setPower(-TURN_SPEED);
        backLeftMotor.setPower(TURN_SPEED);
        backRightMotor.setPower(-TURN_SPEED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.28)) {   // 300 milliseconds
            telemetry.addData("Path", "Turning to goal: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        // Stop
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);

        sleep(200);

        // -------------------------
        // 2. SHOOT FIRST TWO BALLS
        // -------------------------
        shooterMotor.setPower(0.9);
        sleep(1000); // spin-up
        intakeMotor.setPower(-1);
        sleep(200);  // feed bal
        intakeMotor.setPower(0);
        sleep(1200);  // shooter
        intakeMotor.setPower(-1);
        sleep(300); // feed ball
        intakeMotor.setPower(0);
        shooterMotor.setPower(0);
        sleep(150);

        // Move off launch line (strafe right)
        frontLeftMotor.setPower(FORWARD_SPEED);
        frontRightMotor.setPower(-FORWARD_SPEED);
        backLeftMotor.setPower(-FORWARD_SPEED);
        backRightMotor.setPower(FORWARD_SPEED);

        runtime.reset();
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