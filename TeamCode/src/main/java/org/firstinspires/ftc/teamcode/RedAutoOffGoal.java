package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Red Auto - Main", group = "Red")
public class RedAutoOffGoal extends LinearOpMode {

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
        public void runOpMode() {

            // Initialize the drive system variables.
            frontLeftMotor = hardwareMap.get(DcMotor.class, "FL");
            backLeftMotor = hardwareMap.get(DcMotor.class, "BL");
            frontRightMotor = hardwareMap.get(DcMotor.class, "FR");
            backRightMotor = hardwareMap.get(DcMotor.class, "BR");

            // Initialize the shooting motor and reload-kickstand servo.
            shooterMotor = hardwareMap.get(DcMotor.class, "SM");
            servo = hardwareMap.get(Servo.class, "servo");

            // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
            // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
            // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
            frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
            backRightMotor.setDirection(DcMotor.Direction.FORWARD);

            shooterMotor.setDirection(DcMotor.Direction.FORWARD); // untested
            servo.setDirection(Servo.Direction.FORWARD); // untested

            // Send telemetry message to signify robot waiting;
            telemetry.addData("Status", "Ready to run");
            telemetry.update();

            // Wait for the game to start (driver presses START)
            waitForStart();

            // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.

            // Move to launch position
            frontLeftMotor.setPower(FORWARD_SPEED);
            backLeftMotor.setPower(FORWARD_SPEED);
            frontRightMotor.setPower(FORWARD_SPEED);
            backRightMotor.setPower(FORWARD_SPEED);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.0)) {
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

            // Shoot !3! artifacts
            shooterMotor.setPower(1);
            for (int i = 1; i < 3; i++) {
                sleep(500); // untested
                servo.setPosition(0.5);
                sleep(500);
                servo.setPosition(0);
            }
            shooterMotor.setPower(0);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3.0)) {
                telemetry.addData("Path", "Shooting...: %4.1f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Move off of launch line (right strafe)
            frontLeftMotor.setPower(FORWARD_SPEED);
            backLeftMotor.setPower(-FORWARD_SPEED);
            frontRightMotor.setPower(-FORWARD_SPEED);
            backRightMotor.setPower(FORWARD_SPEED);

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
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

            // current code = 6 second auto
        }



    }
