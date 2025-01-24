package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RoadrunnerTest extends LinearOpMode {
    // systems
    private PinpointDrive drive;
    private ArmSubsystemRoadrunner armSubsystem;

    // pose2ds
    private Pose2d startPose = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode(){
        drive = new PinpointDrive(hardwareMap, startPose);

        ParallelAction action = new ParallelAction(
                drive.actionBuilder(startPose)
                        .lineToX(24)
                        .build(),
                armSubsystem.armTo(1, 5000)
        );
        waitForStart();

        Actions.runBlocking(action);
    }
}
