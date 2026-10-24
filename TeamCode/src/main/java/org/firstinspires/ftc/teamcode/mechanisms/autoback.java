package org.firstinspires.ftc.teamcode.mechanisms;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="auto-back")
public class autoback extends LinearOpMode {
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
        intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        if (isStopRequested()) {
            return;
        }
        sleep(15000);
        frontLeftMotor.setPower(STRAFE_SPEED);
        frontRightMotor.setPower(-STRAFE_SPEED);
        backLeftMotor.setPower(-STRAFE_SPEED);
        backRightMotor.setPower(STRAFE_SPEED);
        sleep(300);
        stopDrive(frontLeftMotor, backLeftMotor,
                frontRightMotor, backRightMotor);
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
