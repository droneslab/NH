
"""
Support for FLIR cameras using Spinnaker SDK.
This module provides functions to configure camera settings, capture images,
and manage camera operations using the PySpin library.
"""

import PySpin
import numpy as np
import cv2

def log(value, level="DONT"):
    # if level == "INFO":
    #     log (value) # FIXME :  Fix to print debug
    print (value)


def configure_camera_target_value_manual(cam) -> PySpin.CameraPtr:
    """
    Configure camera settings:
      - Turn off AutoExposureTargetGreyValueAuto
      - Set ExposureAuto to Continuous
      - Set GainAuto to Continuous
    """
    
    nodemap = cam.GetNodeMap()
    
    # ----- Set Exposure to Auto (Continuous) -----
    exposure_auto = PySpin.CEnumerationPtr(nodemap.GetNode("ExposureAuto"))
    if PySpin.IsAvailable(exposure_auto) and PySpin.IsWritable(exposure_auto):
        exposure_auto_continuous = exposure_auto.GetEntryByName("Continuous")
        if PySpin.IsAvailable(exposure_auto_continuous) and PySpin.IsReadable(exposure_auto_continuous):
            exposure_auto.SetIntValue(exposure_auto_continuous.GetValue())
            log("Auto Exposure turned on (Continuous mode).")
        else:
            log("Unable to set ExposureAuto to Continuous.")
    else:
        log("ExposureAuto not available or not writable.")
    
    # ----- Set Gain to Auto (Continuous) -----
    gain_auto = PySpin.CEnumerationPtr(nodemap.GetNode("GainAuto"))
    if PySpin.IsAvailable(gain_auto) and PySpin.IsWritable(gain_auto):
        gain_auto_continuous = gain_auto.GetEntryByName("Continuous")
        if PySpin.IsAvailable(gain_auto_continuous) and PySpin.IsReadable(gain_auto_continuous):
            gain_auto.SetIntValue(gain_auto_continuous.GetValue())
            log("Auto Gain turned on (Continuous mode).")
        else:
            log("Unable to set GainAuto to Continuous.")
    else:
        log("GainAuto not available or not writable.")


    # ----- Disable AutoExposureTargetGreyValueAuto -----
    grey_value_auto = PySpin.CEnumerationPtr(nodemap.GetNode("AutoExposureTargetGreyValueAuto"))
    if PySpin.IsAvailable(grey_value_auto) and PySpin.IsWritable(grey_value_auto):
        grey_value_auto_off = grey_value_auto.GetEntryByName("Off")
        if PySpin.IsAvailable(grey_value_auto_off) and PySpin.IsReadable(grey_value_auto_off):
            grey_value_auto.SetIntValue(grey_value_auto_off.GetValue())
            log("AutoExposureTargetGreyValueAuto turned off.")
        else:
            log("Unable to set AutoExposureTargetGreyValueAuto to Off.")
    else:
        log("AutoExposureTargetGreyValueAuto not available or not writable.")
    
    return cam

def configure_camera_target_value(cam, target_value: int) -> PySpin.CameraPtr:
    """
    Configure camera settings:
      - Set AutoExposureTargetGreyValue to the target value (0-99)
    """
    if not (0 <= target_value <= 99):
        log("Target value must be between 0 and 99.")
        return cam
    
    nodemap = cam.GetNodeMap()
    
    # ----- Set AutoExposureTargetGreyValue -----
    grey_value_node = PySpin.CFloatPtr(nodemap.GetNode("AutoExposureTargetGreyValue"))
    if PySpin.IsAvailable(grey_value_node) and PySpin.IsWritable(grey_value_node):
        grey_value_node.SetValue(float(target_value))
        log("AutoExposureTargetGreyValue set to {}.".format(target_value))
    else:
        log("Unable to set AutoExposureTargetGreyValue.")
    
    return cam


def configure_camera_auto(cam) -> PySpin.CameraPtr:
    """
    Configure camera settings to use automatic gain and exposure:
      - Turn on auto exposure (set to Continuous mode)
      - Turn on auto gain (set to Continuous mode)
    """
    nodemap = cam.GetNodeMap()

    # ----- Exposure Settings -----
    exposure_auto = PySpin.CEnumerationPtr(nodemap.GetNode("ExposureAuto"))
    if PySpin.IsAvailable(exposure_auto) and PySpin.IsWritable(exposure_auto):
        # Set auto exposure to Continuous mode
        exposure_auto_continuous = exposure_auto.GetEntryByName("Continuous")
        if PySpin.IsAvailable(exposure_auto_continuous) and PySpin.IsReadable(exposure_auto_continuous):
            exposure_auto.SetIntValue(exposure_auto_continuous.GetValue())
            log("Auto Exposure turned on (Continuous mode).")
        else:
            log("Unable to set ExposureAuto to Continuous.")
    else:
        log("ExposureAuto not available or not writable.")

    # ----- Gain Settings -----
    gain_auto = PySpin.CEnumerationPtr(nodemap.GetNode("GainAuto"))
    if PySpin.IsAvailable(gain_auto) and PySpin.IsWritable(gain_auto):
        # Set auto gain to Continuous mode
        gain_auto_continuous = gain_auto.GetEntryByName("Continuous")
        if PySpin.IsAvailable(gain_auto_continuous) and PySpin.IsReadable(gain_auto_continuous):
            gain_auto.SetIntValue(gain_auto_continuous.GetValue())
            log("Auto Gain turned on (Continuous mode).")
        else:
            log("Unable to set GainAuto to Continuous.")
    else:
        log("GainAuto not available or not writable.")

    return cam

def configure_camera(cam, exposure_time_ms=20.0, gain_value=12) -> PySpin.CameraPtr:
    """
    Configure camera settings:
      - Turn off auto exposure and set exposure mode to "Timed"
      - Set exposure time (in microseconds)
      - Turn off auto gain and set gain value (in dB)
    """
    exposure_time = exposure_time_ms*1000
    nodemap = cam.GetNodeMap()

    # ----- Exposure Settings -----
    # Turn off auto exposure
    exposure_auto = PySpin.CEnumerationPtr(nodemap.GetNode("ExposureAuto"))
    if PySpin.IsAvailable(exposure_auto) and PySpin.IsWritable(exposure_auto):
        exposure_auto_off = exposure_auto.GetEntryByName("Off")
        if PySpin.IsAvailable(exposure_auto_off) and PySpin.IsReadable(exposure_auto_off):
            exposure_auto.SetIntValue(exposure_auto_off.GetValue())
            log("Auto Exposure turned off.")
        else:
            log("Unable to set ExposureAuto to Off.")
    else:
        log("ExposureAuto not available or not writable.")

    # Set exposure mode to "Timed"
    exposure_mode = PySpin.CEnumerationPtr(nodemap.GetNode("ExposureMode"))
    if PySpin.IsAvailable(exposure_mode) and PySpin.IsWritable(exposure_mode):
        exposure_mode_timed = exposure_mode.GetEntryByName("Timed")
        if PySpin.IsAvailable(exposure_mode_timed) and PySpin.IsReadable(exposure_mode_timed):
            exposure_mode.SetIntValue(exposure_mode_timed.GetValue())
            log("Exposure mode set to Timed.")
        else:
            log("Unable to set ExposureMode to Timed.")
    else:
        log("ExposureMode not available or not writable.")

    # Set exposure time (in microseconds)
    exposure_time_node = PySpin.CFloatPtr(nodemap.GetNode("ExposureTime"))
    if PySpin.IsAvailable(exposure_time_node) and PySpin.IsWritable(exposure_time_node):
        exposure_time_node.SetValue(exposure_time)
        log("Exposure time set to {} ms.".format(exposure_time_ms))
    else:
        log("Unable to set ExposureTime.")

    # ----- Gain Settings -----
    # Turn off auto gain
    gain_auto = PySpin.CEnumerationPtr(nodemap.GetNode("GainAuto"))
    if PySpin.IsAvailable(gain_auto) and PySpin.IsWritable(gain_auto):
        gain_auto_off = gain_auto.GetEntryByName("Off")
        if PySpin.IsAvailable(gain_auto_off) and PySpin.IsReadable(gain_auto_off):
            gain_auto.SetIntValue(gain_auto_off.GetValue())
            log("Auto Gain turned off.")
        else:
            log("Unable to set GainAuto to Off.")
    else:
        log("GainAuto not available or not writable.")

    # Set gain value (in dB)
    gain_node = PySpin.CFloatPtr(nodemap.GetNode("Gain"))
    if PySpin.IsAvailable(gain_node) and PySpin.IsWritable(gain_node):
        gain_node.SetValue(gain_value)
        log("Gain set to {} dB.".format(gain_value))
    else:
        log("Unable to set Gain.")

    # ----- Pixel Format for Color Image -----
    # Set pixel format to BGR8 for color image capture
    pixel_format = PySpin.CEnumerationPtr(nodemap.GetNode("PixelFormat"))
    if PySpin.IsAvailable(pixel_format) and PySpin.IsWritable(pixel_format):
        pixel_format_bgr8 = pixel_format.GetEntryByName("BGR8")
        if PySpin.IsAvailable(pixel_format_bgr8) and PySpin.IsReadable(pixel_format_bgr8):
            pixel_format.SetIntValue(pixel_format_bgr8.GetValue())
            log("Pixel format set to BGR8 for color capture.")
        else:
            log("BGR8 pixel format not available.")
    else:
        log("PixelFormat not available or not writable.")

def capture_image_rolling(cam, resize: tuple = None) -> np.ndarray:
    """
    Captures an image from the camera, converts it to a NumPy array (cv2 image),
    and optionally resizes it.

    Optimizations:
    - Minimizes redundant checks
    - Uses `try-finally` to ensure acquisition is stopped properly
    - Optimizes `cv2.resize()` performance
    """
    nodemap = cam.GetNodeMap()

    # Set acquisition mode to Continuous only if necessary
    # acquisition_mode = PySpin.CEnumerationPtr(nodemap.GetNode("AcquisitionMode"))
    # if PySpin.IsAvailable(acquisition_mode) and PySpin.IsWritable(acquisition_mode):
    #     acquisition_mode_continuous = acquisition_mode.GetEntryByName("Continuous")
    #     if PySpin.IsAvailable(acquisition_mode_continuous) and PySpin.IsReadable(acquisition_mode_continuous):
    #         if acquisition_mode.GetIntValue() != acquisition_mode_continuous.GetValue():
    #             acquisition_mode.SetIntValue(acquisition_mode_continuous.GetValue())
    #             log("Acquisition mode set to Continuous.")

    # cam.BeginAcquisition()
    log("Acquiring image...")

    try:
        image_result = cam.GetNextImage()

        if image_result.IsIncomplete():
            log(f"Image incomplete with status {image_result.GetImageStatus()}.")
            return None

        # Convert to NumPy array (cv2 image)
        image_data = image_result.GetNDArray()
        
    finally:
        # Always release image and stop acquisition
        image_result.Release()
        # cam.EndAcquisition()
        log("Image acquired.")

    # Optimize resize performance
    if resize:
        # image_data = cv2.resize(image_data, resize, interpolation=cv2.INTER_AREA)
        image_data = cv2.resize(image_data, (resize[0],resize[1]))


    return image_data

def capture_image(cam, resize:tuple=None) -> np.ndarray:
    """
    Sets acquisition mode, starts acquisition, grabs one image,
    and returns it as a NumPy array (cv2 image).
    """
    nodemap = cam.GetNodeMap()

    # Set acquisition mode to Continuous
    acquisition_mode = PySpin.CEnumerationPtr(nodemap.GetNode("AcquisitionMode"))
    if PySpin.IsAvailable(acquisition_mode) and PySpin.IsWritable(acquisition_mode):
        acquisition_mode_continuous = acquisition_mode.GetEntryByName("Continuous")
        if PySpin.IsAvailable(acquisition_mode_continuous) and PySpin.IsReadable(acquisition_mode_continuous):
            acquisition_mode.SetIntValue(acquisition_mode_continuous.GetValue())
            log("Acquisition mode set to Continuous.")
        else:
            log("Unable to set AcquisitionMode to Continuous.")
    else:
        log("AcquisitionMode not available or not writable.")

    cam.BeginAcquisition()
    log("Acquiring image...")

    image_result = cam.GetNextImage()

    if image_result.IsIncomplete():
        log("Image incomplete with image status {}.".format(image_result.GetImageStatus()))
        image_result.Release()
        cam.EndAcquisition()
        return None
    else:
        # Convert the image to a NumPy array (cv2 image)
        image_data = image_result.GetNDArray()
        image_result.Release()
        cam.EndAcquisition()
        log("Image acquired.")

        if resize is not None:
            image_data = cv2.resize(image_data, (resize[0],resize[1]))

        return image_data

def main():
    # Retrieve singleton reference to the system object and camera list
    system = PySpin.System.GetInstance()
    cam_list = system.GetCameras()

    num_cameras = cam_list.GetSize()
    log("Number of cameras detected: {}".format(num_cameras))

    if num_cameras == 0:
        cam_list.Clear()
        system.ReleaseInstance()
        log("No cameras detected!")
        return

    # Use the first available camera
    cam = cam_list.GetByIndex(0)

    try:
        cam.Init()

        # Configure the camera with desired exposure and gain values
        configure_camera(cam, exposure_time_ms=50.0, gain_value=12)

        log (type(cam))

        # Capture an image
        image = capture_image(cam)

        if image is not None:
            # Display the image using OpenCV
            # cv2.imshow("Captured Image", image)
            # log("Press any key to close the image window.")
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
            # Optionally, save the image
            cv2.imwrite("captured_image.png", image)
            log (type(image))
        else:
            log("Failed to capture a valid image.")

        cam.DeInit()

    except PySpin.SpinnakerException as ex:
        log("Error: {}".format(ex))

    # Clean up
    del cam
    cam_list.Clear()
    system.ReleaseInstance()

if __name__ == '__main__':
    main()