/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
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

package teamcode.javalimelight.trclib.command;

import teamcode.javalimelight.trclib.controller.TrcPidController;
import teamcode.javalimelight.trclib.drivebase.TrcDriveBase;
import teamcode.javalimelight.trclib.pathdrive.TrcPath;
import teamcode.javalimelight.trclib.pathdrive.TrcPose2D;
import teamcode.javalimelight.trclib.pathdrive.TrcPurePursuitDrive;
import teamcode.javalimelight.trclib.robotcore.TrcEvent;
import teamcode.javalimelight.trclib.robotcore.TrcRobot;

/**
 * This class implements a generic Pure Pursuit Drive command. It allows the caller to specify the drive path by
 * calling start with an array of Waypoint poses.
 */
public class CmdPurePursuitDrive implements TrcRobot.RobotCommand
{
    private static final double DEF_FOLLOWING_DISTANCE = 6.0;
    private static final double DEF_POS_TOLERANCE = 2.0;
    private static final double DEF_TURN_TOLERANCE = 1.0;

    private final TrcPurePursuitDrive purePursuitDrive;
    private final TrcEvent event;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param driveBase specifies the drive base object.
     * @param followingDistance specifies the following distance.
     * @param posTolerance specifies the position tolerance
     * @param turnTolerance specifies the turn tolerance.
     * @param xPosPidCoeff specifies the PID coefficients for X position PID controller.
     * @param yPosPidCoeff specifies the PID coefficients for Y position PID controller.
     * @param turnPidCoeff specifies the PID coefficients for turn PID controller.
     * @param velPidCoeff specifies the PID coefficients for velocity PID controller.
     */
    public CmdPurePursuitDrive(
        TrcDriveBase driveBase, double followingDistance, double posTolerance, double turnTolerance,
        TrcPidController.PidCoefficients xPosPidCoeff, TrcPidController.PidCoefficients yPosPidCoeff,
        TrcPidController.PidCoefficients turnPidCoeff, TrcPidController.PidCoefficients velPidCoeff)
    {
        final String moduleName = getClass().getSimpleName();

        purePursuitDrive = new TrcPurePursuitDrive(
            moduleName, driveBase, followingDistance, posTolerance, turnTolerance,
            xPosPidCoeff, yPosPidCoeff, turnPidCoeff, velPidCoeff);
        event = new TrcEvent(moduleName);
    }   //CmdPurePursuitDrive

    /**
     * Constructor: Create an instance of the object.
     *
     * @param driveBase specifies the drive base object.
     * @param xPosPidCoeff specifies the PID coefficients for X position PID controller.
     * @param yPosPidCoeff specifies the PID coefficients for Y position PID controller.
     * @param turnPidCoeff specifies the PID coefficients for turn PID controller.
     * @param velPidCoeff specifies the PID coefficients for velocity PID controller.
     */
    public CmdPurePursuitDrive(
        TrcDriveBase driveBase, TrcPidController.PidCoefficients xPosPidCoeff,
        TrcPidController.PidCoefficients yPosPidCoeff, TrcPidController.PidCoefficients turnPidCoeff,
        TrcPidController.PidCoefficients velPidCoeff)
    {
        this(driveBase, DEF_FOLLOWING_DISTANCE, DEF_POS_TOLERANCE, DEF_TURN_TOLERANCE, xPosPidCoeff, yPosPidCoeff,
             turnPidCoeff, velPidCoeff);
    }   //CmdPurePursuitDrive

    /**
     * This method starts the Pure Pursuit drive with the specified drive path.
     *
     * @param path specifies the drive path with waypoints.
     * @param timeout specifies the maximum time allowed for this operation.
     * @param maxVel specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     */
    public void start(TrcPath path, double timeout, Double maxVel, Double maxAccel)
    {
        purePursuitDrive.start(path, event, timeout, maxVel, maxAccel);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified drive path.
     *
     * @param path specifies the drive path with waypoints.
     * @param maxVel specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     */
    public void start(TrcPath path, Double maxVel, Double maxAccel)
    {
        purePursuitDrive.start(path, event, 0.0, maxVel, maxAccel);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified drive path.
     *
     * @param path specifies the drive path with waypoints.
     * @param timeout specifies the maximum time allowed for this operation.
     */
    public void start(TrcPath path, double timeout)
    {
        purePursuitDrive.start(path, event, timeout, null, null);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified drive path.
     *
     * @param path specifies the drive path with waypoints.
     */
    public void start(TrcPath path)
    {
        purePursuitDrive.start(path, event, 0.0, null, null);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param timeout specifies the maximum time allowed for this operation.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *        false if appending point is in the same reference frame as startingPose.
     * @param maxVel specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        double timeout, boolean incrementalPath, Double maxVel, Double maxAccel, Double maxDecel, TrcPose2D... poses)
    {
        purePursuitDrive.start(event, timeout, incrementalPath, maxVel, maxAccel, maxDecel, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses read either from the built-in resources
     * or from a file.
     *
     * @param timeout specifies the maximum time allowed for this operation.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *        false if appending point is in the same reference frame as startingPose.
     * @param maxVel specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param maxDecel specifies the maximum deceleration if applying trapezoid velocity profile, null if not.
     * @param path specifies the file system path or resource name.
     * @param loadFromResources specifies true if the data is from attached resources, false if from file system.
     */
    public void start(
        double timeout, boolean incrementalPath, Double maxVel, Double maxAccel, Double maxDecel,
        String path, boolean loadFromResources)
    {
        purePursuitDrive.start(
            event, timeout, incrementalPath, maxVel, maxAccel, maxDecel, path, loadFromResources);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses read either from the built-in resources
     * or from a file.
     *
     * @param timeout specifies the maximum time allowed for this operation.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *        false if appending point is in the same reference frame as startingPose.
     * @param path specifies the file system path or resource name.
     * @param loadFromResources specifies true if the data is from attached resources, false if from file system.
     */
    public void start(
        double timeout, boolean incrementalPath, String path, boolean loadFromResources)
    {
        purePursuitDrive.start(
            event, timeout, incrementalPath, null, null, null, path, loadFromResources);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *        false if appending point is in the same reference frame as startingPose.
     * @param maxVel specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param maxDecel specifies the maximum deceleration if applying trapezoid velocity profile, null if not.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        boolean incrementalPath, Double maxVel, Double maxAccel, Double maxDecel, TrcPose2D... poses)
    {
        purePursuitDrive.start(event, 0.0, incrementalPath, maxVel, maxAccel, maxDecel, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param timeout specifies the maximum time allowed for this operation.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *        false if appending point is in the same reference frame as startingPose.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(double timeout, boolean incrementalPath, TrcPose2D... poses)
    {
        purePursuitDrive.start(event, timeout, incrementalPath, null, null, null, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *        false if appending point is in the same reference frame as startingPose.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(boolean incrementalPath, TrcPose2D... poses)
    {
        purePursuitDrive.start(event, 0.0, incrementalPath, null, null, null, poses);
    }   //start

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        return purePursuitDrive.isActive();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        purePursuitDrive.cancel();
    }   //cancel

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        return event.isSignaled();
    }   //cmdPeriodic

}   //CmdPurePursuitDrive
