/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.javalimelight.trclib.drivebase;

import teamcode.javalimelight.trclib.robotcore.TrcEvent;
import teamcode.javalimelight.trclib.sensor.TrcGyro;
import teamcode.javalimelight.trclib.motor.TrcMotor;
import teamcode.javalimelight.trclib.sensor.TrcOdometrySensor;
import teamcode.javalimelight.trclib.dataprocessor.TrcUtil;
import teamcode.javalimelight.trclib.timer.TrcTimer;

/**
 * This class implements a platform independent mecanum drive base. A mecanum drive base consists of 4 motor driven
 * wheels. It extends the TrcSimpleDriveBase class so it inherits all the SimpleDriveBase methods and features.
 */
public class TrcMecanumDriveBase extends TrcSimpleDriveBase
{
    private final String moduleName = getClass().getSimpleName();

    /**
     * Constructor: Create an instance of the 4-wheel mecanum drive base.
     *
     * @param lfMotor specifies the left front motor of the drive base.
     * @param lbMotor specifies the left back motor of the drive base.
     * @param rfMotor specifies the right front motor of the drive base.
     * @param rbMotor specifies the right back motor of the drive base.
     * @param gyro specifies the gyro. If none, it can be set to null.
     */
    public TrcMecanumDriveBase(TrcMotor lfMotor, TrcMotor lbMotor, TrcMotor rfMotor, TrcMotor rbMotor, TrcGyro gyro)
    {
        super(lfMotor, lbMotor, rfMotor, rbMotor, gyro);
    }   //TrcMecanumDriveBase

    /**
     * Constructor: Create an instance of the 4-wheel mecanum drive base.
     *
     * @param lfMotor specifies the left front motor of the drive base.
     * @param lbMotor specifies the left back motor of the drive base.
     * @param rfMotor specifies the right front motor of the drive base.
     * @param rbMotor specifies the right back motor of the drive base.
     */
    public TrcMecanumDriveBase(TrcMotor lfMotor, TrcMotor lbMotor, TrcMotor rfMotor, TrcMotor rbMotor)
    {
        super(lfMotor, lbMotor, rfMotor, rbMotor, null);
    }   //TrcMecanumDriveBase

    /**
     * This method checks if it supports holonomic drive.
     *
     * @return true if this drive base supports holonomic drive, false otherwise.
     */
    @Override
    public boolean supportsHolonomicDrive()
    {
        return true;
    }   //supportsHolonomicDrive

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *              ownership aware.
     * @param x specifies the x power.
     * @param y specifies the y power.
     * @param rotation specifies the rotating power.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     * @param gyroAngle specifies the gyro angle to maintain.
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    @Override
    public void holonomicDrive(
        String owner, double x, double y, double rotation, boolean inverted, double gyroAngle, double driveTime,
        TrcEvent event)
    {
        tracer.traceDebug(
            moduleName,
            "owner=" + owner +
            ",x=" + x +
            ",y=" + y +
            ",rot=" + rotation +
            ",inverted=" + inverted +
            ",angle=" + gyroAngle +
            ",driveTime=" + driveTime +
            ",event=" + event);
        if (validateOwnership(owner))
        {
            x = TrcUtil.clipRange(x);
            y = TrcUtil.clipRange(y);
            rotation = TrcUtil.clipRange(rotation);

            if (inverted)
            {
                x = -x;
                y = -y;
            }

            double cosA = Math.cos(Math.toRadians(gyroAngle));
            double sinA = Math.sin(Math.toRadians(gyroAngle));
            double x1 = x*cosA - y*sinA;
            double y1 = x*sinA + y*cosA;

            if (gyroAngle == 0.0 && isGyroAssistEnabled())
            {
                rotation += getGyroAssistPower(rotation);
            }

            if (isAntiTippingEnabled())
            {
                x1 += getAntiTippingPower(true);
                y1 += getAntiTippingPower(false);
            }

            double maxMag = Math.max(Math.abs(x1), Math.abs(y1));
            if (maxMag > 1.0)
            {
                x1 /= maxMag;
                y1 /= maxMag;
            }

            double[] wheelPowers = new double[4];
            wheelPowers[MotorType.LEFT_FRONT.value] = x1 + y1 + rotation;
            wheelPowers[MotorType.RIGHT_FRONT.value] = -x1 + y1 - rotation;
            wheelPowers[MotorType.LEFT_BACK.value] = -x1 + y1 + rotation;
            wheelPowers[MotorType.RIGHT_BACK.value] = x1 + y1 - rotation;
            TrcUtil.normalizeInPlace(wheelPowers);

            double wheelPower;
            boolean wheelsPowered = false;

            wheelPower = wheelPowers[MotorType.LEFT_FRONT.value];
            if (motorPowerMapper != null)
            {
                wheelPower = motorPowerMapper.translateMotorPower(wheelPower, lfMotor.getVelocity());
            }
            lfMotor.setPower(wheelPower);
            if (wheelPower != 0.0) wheelsPowered = true;

            wheelPower = wheelPowers[MotorType.RIGHT_FRONT.value];
            if (motorPowerMapper != null)
            {
                wheelPower = motorPowerMapper.translateMotorPower(wheelPower, rfMotor.getVelocity());
            }
            rfMotor.setPower(wheelPower);
            if (wheelPower != 0.0) wheelsPowered = true;

            wheelPower = wheelPowers[MotorType.LEFT_BACK.value];
            if (motorPowerMapper != null)
            {
                wheelPower = motorPowerMapper.translateMotorPower(wheelPower, lbMotor.getVelocity());
            }
            lbMotor.setPower(wheelPower);
            if (wheelPower != 0.0) wheelsPowered = true;

            wheelPower = wheelPowers[MotorType.RIGHT_BACK.value];
            if (motorPowerMapper != null)
            {
                wheelPower = motorPowerMapper.translateMotorPower(wheelPower, rbMotor.getVelocity());
            }
            rbMotor.setPower(wheelPower);
            if (wheelPower != 0.0) wheelsPowered = true;

            setDriveTime(owner, driveTime, event);

            if (!wheelsPowered)
            {
                // reset stall start time to zero if drive base is stopped.
                stallStartTime = 0.0;
            }
        }
    }   //holonomicDrive

    /**
     * This method is called periodically to calculate the delta between the previous and current motor odometries.
     *
     * @param prevOdometries specifies the previous motor odometries.
     * @param currOdometries specifies the current motor odometries.
     * @return an Odometry object describing the odometry changes since the last update.
     */
    @Override
    protected Odometry getOdometryDelta(
        TrcOdometrySensor.Odometry[] prevOdometries, TrcOdometrySensor.Odometry[] currOdometries)
    {
        //
        // Call super class to get Y and turn data.
        //
        Odometry delta = super.getOdometryDelta(prevOdometries, currOdometries);

        delta.position.x = xScale * TrcUtil.average(
                currOdometries[MotorType.LEFT_FRONT.value].currPos
                - prevOdometries[MotorType.LEFT_FRONT.value].currPos,
                currOdometries[MotorType.RIGHT_BACK.value].currPos
                - prevOdometries[MotorType.RIGHT_BACK.value].currPos,
                -(currOdometries[MotorType.RIGHT_FRONT.value].currPos
                  - prevOdometries[MotorType.RIGHT_FRONT.value].currPos),
                -(currOdometries[MotorType.LEFT_BACK.value].currPos
                  - prevOdometries[MotorType.LEFT_BACK.value].currPos));

        delta.velocity.x = xScale * TrcUtil.average(
                currOdometries[MotorType.LEFT_FRONT.value].velocity,
                currOdometries[MotorType.RIGHT_BACK.value].velocity,
                -currOdometries[MotorType.RIGHT_FRONT.value].velocity,
                -currOdometries[MotorType.LEFT_BACK.value].velocity);

        if (Math.abs(delta.velocity.x) > stallVelThreshold)
        {
            // reset stall start time to current time if drive base has movement.
            // Note: y movement is taken care of by the superclass TrcSimpleDriveBase.
            stallStartTime = TrcTimer.getCurrentTime();
        }

        return delta;
    }   //getOdometryDelta

}   //class TrcMecanumDriveBase
