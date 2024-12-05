package org.firstinspires.ftc.teamcode.Mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake
{

    Servo leftServo;
    Servo rightServo;

    double inLeftPosition = 1.0;
    double inRightPosition = 0.0;

    double outLeftPosition = 0.5;
    double outRightPosition = 1.0 ;


    public Outtake(HardwareMap hardwareMap)
    {
        leftServo = hardwareMap.get(Servo.class, "left_outtake");
        rightServo = hardwareMap.get(Servo.class, "right_outtake");
    }

    class ServoToIn implements Action
    {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            leftServo.setPosition(inLeftPosition);
            rightServo.setPosition(inRightPosition);
            return false;
        }
    }

    public Action servoToIn()
    {
        return new ServoToIn();
    }

    class ServoToOut implements Action
    {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            leftServo.setPosition(outLeftPosition);
            rightServo.setPosition(outRightPosition);
            return false;
        }
    }

    public Action servoToOut()
    {
        return new ServoToOut();
    }

}


