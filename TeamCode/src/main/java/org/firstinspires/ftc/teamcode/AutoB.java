package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoBTwoSpec")
public class AutoB extends LinearOpMode {
    // systems
    private PinpointDrive drive;
    private ArmSubsystemRoadrunner armSubsystem;
    private WristSubsystemRoadrunner wrist;

    // pose2ds
    private static Pose2d startPose = new Pose2d(0, 0, 0);
    private static int ArmSub = 1800;


    @Override
    public void runOpMode(){
        drive = new PinpointDrive(hardwareMap, startPose);
        armSubsystem = new ArmSubsystemRoadrunner(hardwareMap, telemetry);
        wrist = new WristSubsystemRoadrunner(hardwareMap,telemetry);


        waitForStart();
        while (opModeIsActive()){
            Actions.runBlocking(new SequentialAction(
                    // run this, in parallel
//                    new ParallelAction(
//
////                            wrist.wristDown(),
////                            wrist.clawClose(),
//                            drive.actionBuilder(startPose)
////                                    .strafeToConstantHeading(new Vector2d(2,3))
////                                    .strafeToConstantHeading(new Vector2d(11,1))
//
//                                    .build()
////                            armSubsystem.extendTo(1, 7000),
////                            armSubsystem.armTo(1, 1700),
////
//                    ),
//                    armSubsystem.armTo(1, 2000),
//                    armSubsystem.extendTo(1,2000)
//                    armSubsystem.armTo(1, 0),
//                    armSubsystem.extendTo(1,0),
                    wrist.clawOpen(),
                    drive.actionBuilder(startPose)
//                            .strafeToConstantHeading(new Vector2d(-3,0))
//                            .strafeToConstantHeading(new Vector2d(0,-5))
//                            .strafeToConstantHeading(new Vector2d(14,0))
                            .turnTo(Math.toRadians(180))//180 degrees
//                            .strafeToConstantHeading(new Vector2d(0,-3))
//                            .strafeToConstantHeading(new Vector2d(-14,0))
                            .build()


                    //figure out way to stop
            ));
        }
    }
}
