package frc.robot.utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

/**
 * A class for initializing and setting up a USB camera for use with the Robot.
 */
public class Camera {
    /**
     * The USB camera object.
     */
    private UsbCamera camera;

    /**
     * Constructs a new Camera object with the specified device input, resolution
     * width, and resolution height.
     * 
     * @param deviceInput The USB port number of the camera.
     * @param resW        The desired resolution width of the camera feed.
     * @param resH        The desired resolution height of the camera feed.
     */
    public Camera(int deviceInput, int resW, int resH) {
        // Starts automatic capture of camera with the given device input
        camera = CameraServer.startAutomaticCapture(deviceInput);

        // Sets the resolution of the camera feed
        camera.setResolution(resW, resH);
    }
}
