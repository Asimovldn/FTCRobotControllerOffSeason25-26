package org.firstinspires.ftc.teamcode.Auxiliar;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Encoder
{

    
    DcMotorEx encoderMotorPort;

    public Encoder(DcMotorEx encoderMotorPort)
    {
        this.encoderMotorPort = encoderMotorPort;
    }

    public int getPosition()
    {
        return encoderMotorPort.getCurrentPosition();

    }

    public double getAngularVelocity(AngleUnit angleUnit)
    {
        return encoderMotorPort.getVelocity(angleUnit);
    }





}
