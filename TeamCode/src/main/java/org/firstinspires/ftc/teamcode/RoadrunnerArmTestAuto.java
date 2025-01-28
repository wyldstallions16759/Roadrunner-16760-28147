package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="ARMSUBSYSTEST")
public class RoadrunnerArmTestAuto extends LinearOpMode {
    @Override
    public void runOpMode(){
        ArmSubsystemRoadrunner arm = new ArmSubsystemRoadrunner(hardwareMap, telemetry);

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        arm.extendTo(1, 8000)
                )
        );
    }
}
