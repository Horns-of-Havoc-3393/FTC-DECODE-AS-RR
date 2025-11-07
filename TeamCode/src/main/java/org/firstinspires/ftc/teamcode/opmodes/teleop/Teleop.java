package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.funcs.SlewRateLimiter;

@TeleOp(name="Teleop", group="Compcode")
public class Teleop extends LinearOpMode {
    //boolean shoot = false;
    //boolean lastShot = false;
    //boolean currentshot = false;
    int speed = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        SlewRateLimiter lim = new SlewRateLimiter(500);
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class,"flm");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class,"blm");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class,"frm");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "brm");
        DcMotorEx shooterR = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx shooterL = hardwareMap.get(DcMotorEx.class, "shooter2");

        DcMotorEx belt = hardwareMap.get(DcMotorEx.class, "convayer");
        DcMotorEx inta = hardwareMap.get(DcMotorEx.class, "rubbers");
        Servo flipper = hardwareMap.get(Servo.class, "flipper");


        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        shooterR.setVelocity(lim.calculate(1800));
        shooterL.setVelocity(lim.calculate(1800));
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            belt.setVelocity(1000);
            inta.setVelocity(1000);
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            if(gamepad1.right_bumper){
                flipper.setPosition(1);
            }else {
                flipper.setPosition(0);
            }

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) ;
            double backLeftPower = (rotY - rotX + rx) ;
            double frontRightPower = (rotY - rotX - rx) ;
            double backRightPower = (rotY + rotX - rx) ;

            frontLeftMotor.setVelocity(2800*frontLeftPower);
            backLeftMotor.setVelocity(2800*backLeftPower);
            frontRightMotor.setVelocity(2800*frontRightPower);
            backRightMotor.setVelocity(2800*backRightPower);
        }
    }
}
