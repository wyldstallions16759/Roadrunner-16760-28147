package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous(name="clawAuto")
public class AutoClaw extends LinearOpMode {
    @Override
    public void runOpMode(){
        WristSubsystemRoadrunner wrist = new WristSubsystemRoadrunner(hardwareMap, telemetry);
        ArmSubsystemRoadrunner arm = new ArmSubsystemRoadrunner(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()){
            Actions.runBlocking(
                    new SequentialAction(
                            wrist.clawClose()
                    ));
        }
    }
}
