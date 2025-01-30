package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Pinpoint.Pinpoint;

import java.util.concurrent.TimeUnit;

public class pinpointRoadrunner implements Action {
    private int time;
    private boolean first;
    private ElapsedTime timer;
    private Pinpoint pinpoint;

    public pinpointRoadrunner(LinearOpMode opMode, HardwareMap hardwareMap, Telemetry telemetry) {
        super();
        pinpoint = new Pinpoint(opMode,hardwareMap,telemetry);
    }
    public
    @Override
}
