package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmSubsystemRoadrunner {
    // Motors
    private DcMotor wormGear;
    private DcMotor actuator;

    // Telemetry
    private Telemetry telemetry;

    public ArmSubsystemRoadrunner(HardwareMap hardwareMap, Telemetry telemetry){
        wormGear = hardwareMap.get(DcMotor.class, "Elevation");
        actuator = hardwareMap.get(DcMotor.class, "Extension");
        wormGear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wormGear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        actuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.telemetry = telemetry;
    }

    public class ArmTo implements Action{
        private double speed;
        private int target;
        private int tolerance;

        public ArmTo(double speed, int target){
            super();
            this.speed = speed;
            this.target = target;
            this.tolerance = 100;
        }

        public ArmTo(double speed, int target, int tolerance){
            super();
            this.speed = speed;
            this.target = target;
            this.tolerance = tolerance;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            int position = wormGear.getCurrentPosition();
            telemetry.addData("Arm Position: ", position);
            telemetry.update();
            if (Math.abs(this.target - position) < tolerance){
                wormGear.setPower(0);
                return false;
            }

            if (position < target) {
                wormGear.setPower(speed);
            } else {
                wormGear.setPower(-speed);
            }
            return true;
        }
    }

    public Action armTo(double speed, int target){
        return new ArmTo(speed, -target);
    }
    public Action armTo(double speed, int target, int tolerance){
        return new ArmTo(speed, -target, tolerance);
    }

    public class ExtendTo implements Action{
        private double speed;
        private int target;
        private int tolerance;

        public ExtendTo(double speed, int target){
            super();
            this.speed = speed;
            this.target = target;
            this.tolerance = 1000;
        }

        public ExtendTo(double speed, int target, int tolerance){
            super();
            this.speed = speed;
            this.target = target;
            this.tolerance = tolerance;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            int position = actuator.getCurrentPosition();
            telemetry.addData("Arm Position: ", position);
            telemetry.update();
            if (Math.abs(this.target - position) < tolerance){
                actuator.setPower(0);
                return false;
            }

            if (position > target) {
                actuator.setPower(-speed);
            } else {
                actuator.setPower(speed);
            }
            return true;
        }
    }

    public Action extendTo(double speed, int target){
        return new ExtendTo(speed, target);
    }
    public Action extendTo(double speed, int target, int tolerance){
        return new ExtendTo(speed, target, tolerance);
    }

}
