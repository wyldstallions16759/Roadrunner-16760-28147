package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class Pause implements Action {
    private int time;
    private boolean first;
    private ElapsedTime timer;
    public Pause(int ms){
        super();
        time = ms;
        first = true;

        timer = new ElapsedTime();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet){
        if (first){
            first = false;
            timer.reset();
        }

        return (timer.time(TimeUnit.MILLISECONDS) < time);
    }
}
