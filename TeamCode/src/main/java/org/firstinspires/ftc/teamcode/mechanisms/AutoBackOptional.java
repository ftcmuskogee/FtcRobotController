package org.firstinspires.ftc.teamcode.mechanisms;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="back auto - needs testing")
public class AutoBackOptional extends LinearOpMode {
    static final double FORWARD_SPEED = 0.6;
    static final double STRAFE_SPEED = 0.6;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor =
                hardwareMap.get(DcMotor.class, "FL");
        DcMotor backLeftMotor =
                hardwareMap.get(DcMotor.class, "BL");
        DcMotor frontRightMotor =
                hardwareMap.get(DcMotor.class, "FR");
        DcMotor backRightMotor =
                hardwareMap.get(DcMotor.class, "BR");

        DcMotor intakeMotor =
                hardwareMap.get(DcMotor.class, "NTK");
        DcMotor intakeMotor2 =
                hardwareMap.get(DcMotor.class, "NTK2");
        DcMotor shooterMotor =
                hardwareMap.get(DcMotor.class, "Shooter");

        // Motor directions
        intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake when stopped
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) {
            return;
        }



        // 1. Spin up shooter
        shooterMotor.setPower(-0.75);
        sleep(9000);

        // 2. Feed game pieces into shooter
        intakeMotor.setPower(-1.0);
        intakeMotor2.setPower(-1.0);
        sleep(6000);

        // Stop shooter and intake
        intakeMotor.setPower(0);
        intakeMotor2.setPower(0);
        shooterMotor.setPower(0);

        frontLeftMotor.setPower(FORWARD_SPEED);
        frontRightMotor.setPower(FORWARD_SPEED);
        backLeftMotor.setPower(FORWARD_SPEED);
        backRightMotor.setPower(FORWARD_SPEED);
        sleep(250);

        stopDrive(frontLeftMotor, backLeftMotor,
                frontRightMotor, backRightMotor);

        frontLeftMotor.setPower(STRAFE_SPEED);
        frontRightMotor.setPower(-STRAFE_SPEED);
        backLeftMotor.setPower(-STRAFE_SPEED);
        backRightMotor.setPower(STRAFE_SPEED);

        sleep(650);

        // 5. Stop everything
        stopDrive(frontLeftMotor, backLeftMotor,
                frontRightMotor, backRightMotor);
        intakeMotor.setPower(0);
        intakeMotor2.setPower(0);
        shooterMotor.setPower(0);
    }

    private void stopDrive(
            DcMotor fl, DcMotor bl,
            DcMotor fr, DcMotor br) {

        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }
}


