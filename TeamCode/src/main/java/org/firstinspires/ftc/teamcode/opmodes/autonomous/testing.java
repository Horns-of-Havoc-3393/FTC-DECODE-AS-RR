package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.funcs.SlewRateLimiter;
import org.opencv.core.Mat;


@Autonomous(name = "TestAuto", group = "Compcode")
public class testing extends LinearOpMode{
    //Limelight3A limelight;
    private final SlewRateLimiter Lim = new SlewRateLimiter(500);
    //Servo flipper = hardwareMap.get(Servo.class, "flipper");
    //DcMotorEx shooterR = hardwareMap.get(DcMotorEx.class,"shooter1");
    //DcMotorEx shooterL = hardwareMap.get(DcMotorEx.class,"shooter2");
    //DcMotorEx belt = hardwareMap.get(DcMotorEx.class, "belt");
    //DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");

    @Override
    public void runOpMode(){
        //shooterL.setVelocity(Lim.calculate(1800));
        //shooterR.setVelocity(Lim.calculate(1800));
        //belt.setVelocity(Lim.calculate(2800));
        //intake.setVelocity(Lim.calculate(2800));

        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        //limelight.start(); // This tells Limelight to start looking!
    waitForStart();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,Math.PI));
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(new Vector2d(-30,0), Math.toRadians(270))
                        .waitSeconds(3)
                        .strafeToLinearHeading(new Vector2d(-30,-20),Math.toRadians(270))
                        .waitSeconds(3)
                        .strafeToLinearHeading(new Vector2d(-30,0),Math.toRadians(180))

                        .waitSeconds(3)
                        .strafeToLinearHeading(new Vector2d(-60,0),Math.toRadians(235))


                        .build());

    }
//    public class Flipp implements Action {
//        double pot;
//        Servo serv;
//        public Flipp(Servo s, double p){
//            this.pot = p;
//            this.serv = s;
//        }
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket){
//            serv.setPosition(pot);
//            return true;
//        }
//    }
}