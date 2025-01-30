package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.teamcode.Pinpoint.Pinpoint;

@Autonomous(name="AutoCTwoSpec")
public class AutoC extends LinearOpMode {

    //
    private final double TURN_SCALE_FACTOR = 109.0/360;
    // systems
    private PinpointDrive drive;
    private ArmSubsystemRoadrunner armSubsystem;
    private WristSubsystemRoadrunner wrist;
    private pinpointRoadrunner pinpoint;
    enum stateMachine {
        turn1,
        turn2,
        turn3,
        turn4,
        turn5,
    }
    // pose2ds
    private static Pose2d startPose = new Pose2d(0, 0, 0);
    private static int ArmSub = 1800;


    @Override
    public void runOpMode(){
        drive = new PinpointDrive(hardwareMap, startPose);
        armSubsystem = new ArmSubsystemRoadrunner(hardwareMap, telemetry);
        wrist = new WristSubsystemRoadrunner(hardwareMap,telemetry);
        pinpoint = new pinpointRoadrunner(this,hardwareMap,telemetry);

        waitForStart();
        while (opModeIsActive()){
            Actions.runBlocking(new SequentialAction(
                    // run this, in parallel
                    new ParallelAction(
//
                            wrist.wristDown(),
                            wrist.clawClose(),
                            drive.actionBuilder(startPose)
                                    .turnTo(Math.toRadians(180*TURN_SCALE_FACTOR))//180 degrees
//                                    .strafeToConstantHeading(new Vector2d(2,3))
//                                    .strafeToConstantHeading(new Vector2d(11,1))

                                    .build(),
                            new Pause(20000),
                            armSubsystem.extendTo(1, 7500),
                            armSubsystem.armTo(0.6, 1700)
////
                    ),
                    armSubsystem.extendTo(1,1500),
                            wrist.clawOpen(),
                    new ParallelAction(
                        armSubsystem.armTo(1, 0),
                        armSubsystem.extendTo(1,0),

                        drive.actionBuilder(startPose)
                                .strafeToConstantHeading(new Vector2d(-3,0))
                                .strafeToConstantHeading(new Vector2d(0,-5.3))
                                .strafeToConstantHeading(new Vector2d(11,-0.5))//180 degrees
    //                            .strafeToConstantHeading(new Vector2d(-14,0))
    //                            .endTrajectory()
                                .build()
                    ),
                    pinpoint
                            drive.actionBuilder(startPose)
                                    .strafeToConstantHeading(new Vector2d(0,3))
                                    .strafeToConstantHeading(new Vector2d(14,0.25))
                                    .strafeToConstantHeading(new Vector2d(-5,0.25))
                                    .build()


                    //figure out way to stop
            , new Pause(30000)) // the pause action should "stop" (pause for 30 seconds, auto will time out)
            );

        }
    }
}
