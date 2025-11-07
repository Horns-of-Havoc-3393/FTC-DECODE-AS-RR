package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.funcs.SlewRateLimiter;


@Autonomous(name = "Blue_Auto", group = "Compcode")
public class auto extends LinearOpMode{
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

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(60,-37,Math.PI));
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        //.stopAndAdd(new Flipp(flipper,0))
                        .strafeToLinearHeading(new Vector2d(35,-25), Math.toRadians(270))
                        .strafeTo(new Vector2d(35,-50))
                        .waitSeconds(.5)
                        .strafeTo(new Vector2d(35,-30))
                        .strafeToLinearHeading(new Vector2d(-34,-15), Math.PI/3)
                        .waitSeconds(.1)
                        //.stopAndAdd(new Flipp(flipper,1))
                        .waitSeconds(.5)
                        //.stopAndAdd(new Flipp(flipper,0))

                        .strafeToLinearHeading(new Vector2d(-12,-30),3*Math.PI/2)
                        .strafeTo(new Vector2d(-12,-50))
                        .waitSeconds(.5)
                        .strafeToLinearHeading(new Vector2d(-34,-15), Math.PI/3)
                        //.stopAndAdd(new Flipp(flipper,1))
                        .waitSeconds(.5)
                        //.stopAndAdd(new Flipp(flipper,0))

                        .strafeToLinearHeading(new Vector2d(12,-30),3*Math.PI/2)
                        .strafeTo(new Vector2d(12,-50))
                        .waitSeconds(.5)
                        .strafeToLinearHeading(new Vector2d(-34,-15), Math.PI/3)
                        //.stopAndAdd(new Flipp(flipper,1))
                        .waitSeconds(.5)

                        .strafeToLinearHeading(new Vector2d(58,-57),0)


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