package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WristSubsystemRoadrunner{
	private Servo wrist;
	private Servo claw;

	enum WristState{
		UP,
		DOWN
	}
	enum ClawState{
		CLOSED,
		OPEN
	}

	private WristState wristState = WristState.UP;
	private ClawState clawState = ClawState.OPEN;

	private Telemetry telemetry;

	public WristSubsystemRoadrunner(HardwareMap hardwareMap, Telemetry telemetry){
		wrist = hardwareMap.get(Servo.class, "ShoulderPivot");
		claw = hardwareMap.get(Servo.class, "RightFinger");	

		this.telemetry = telemetry;
		wrist.scaleRange(0.35, 0.7);
		claw.scaleRange(0.4, 0.7);
	}

	public class GenericAction implements Action{
		@Override
		public boolean run(@NonNull TelemetryPacket packet){
			return false; // return false as to do nothing. All actual servo movements will be handled by the class that calls this.
		}
	}
	
	public Action wristUp(){
		// move the wrist to the side
		this.wrist.setPosition(1);
		this.wristState = WristState.UP;
		return new GenericAction();
	}
	
	public Action wristDown(){
		// move the wrist to a downward position
		this.wrist.setPosition(0);
		this.wristState = WristState.DOWN;
		return new GenericAction();
	}

	public Action clawClose(){
		// move the claw closed
		this.claw.setPosition(0);
		this.clawState = ClawState.CLOSED;
		return new GenericAction();
	}

	public Action clawOpen(){
		// open the claw
		this.claw.setPosition(1);
		this.clawState = ClawState.OPEN;
		return new GenericAction();
	}

	public Action toggleWrist(){
		// opens the wrist if it is closed.
		// closes the wrist if it is open.

		if (this.wristState == WristState.DOWN){
			return this.wristUp();
		}
		else{
			return this.wristDown();
		}
	}

	public Action toggleClaw(){
		// opens the claw if it is closed.
		// closes the claw if it is open.

		if (this.clawState == ClawState.CLOSED){
			return this.clawOpen();
		}
		else{
			return this.clawClose();
		}
	}
}







