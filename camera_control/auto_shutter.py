"""
Supports controlling a camera using gphoto2. Tested with Fujifilm X-T3.
"""

import subprocess as sp
import sys, os
import argparse
import cv2


def capture(filename='temp') -> None:
    result = sp.call(["gphoto2", "--capture-image-and-download", f"--filename={filename}.jpg"])
    if result != 0:
        print("Error capturing image", file=sys.stderr)


def set_shutter_speed(speed: int) -> None:
    #  gphoto2 --set-config-value /main/capturesettings/shutterspeed="1/20"
    result = sp.call(["gphoto2", "--set-config-value", f"/main/capturesettings/shutterspeed=1/{str(speed)}"])
    if result != 0:
        print("Error setting shutter speed", file=sys.stderr)

    set_speed = get_shutter_speed()
    print (f"Set shutter speed: {speed}, got {set_speed}")
    if set_speed != speed:
        print(f"Error setting shutter speed: {speed}, got {set_speed}", file=sys.stderr)
        raise Exception("Error setting shutter speed")

def get_shutter_speed() -> int:
    result = sp.run(["gphoto2", "--get-config", "/main/capturesettings/shutterspeed"], capture_output=True, text=True)
    if result.returncode != 0:
        print("Error getting shutter speed", file=sys.stderr)
        return -1

    for line in result.stdout.splitlines():
        if line.startswith("Current:"):
            res = line.split()[-1]
            res = res.split('/')[-1]
            return int(res)
    return -1

def clean_connection() -> None:
    result = sp.call(["pkill", "-f", "gphoto2"])
    if result != 0:
        print("Error cleaning connection", file=sys.stderr)

def get_iso() -> int:
    result = sp.run(["gphoto2", "--get-config", "/main/imgsettings/iso"], capture_output=True, text=True)
    if result.returncode != 0:
        print("Error getting ISO", file=sys.stderr)
        return -1

    for line in result.stdout.splitlines():
        if line.startswith("Current:"):
            return int(line.split()[-1])
    return -1
    
def get_fnumber() -> str:
    result = sp.run(["gphoto2", "--get-config", "/main/capturesettings/f-number"], capture_output=True, text=True)
    if result.returncode != 0:
        print("Error getting f-number", file=sys.stderr)
        return -1

    for line in result.stdout.splitlines():
        if line.startswith("Current:"):
            res = line.split()[-1]
            res = res.split('/')[-1]
            return res
    return -1


def set_and_capture(speed: int, filename='temp') -> None:
    clean_connection()
    set_shutter_speed(speed)
    capture(filename)
    clean_connection()

def set_capture_get(speed: int, filename='temp', resize=None, clean=False) -> cv2.Mat:
    clean_connection()
    set_shutter_speed(speed)
    capture(filename)
    clean_connection()

    image = cv2.imread(f"{filename}.jpg")

    # Delete the file after reading it

    os.remove(f"{filename}.jpg")

    if resize is not None: 
        # Resize is a tuple (width, height)
        print (f"Resizing image to {resize}")
        image = cv2.resize(image, resize)

    # Save the resized image
    print (f"Saving resized image to {filename}_resized.jpg")
    cv2.imwrite(f"{filename}_resized.jpg", image)

    if clean:
        print (f"Deleting resized image {filename}_resized.jpg")
        os.remove(f"{filename}_resized.jpg")

    return image

def loop_capture(speeds, fileprefix='test', ext_light_intensity=None) -> None:
    for i, speed in enumerate(speeds):
        fnumber = get_fnumber()
        iso = get_iso()
        if ext_light_intensity is not None:
            set_and_capture(speed, f'{fileprefix}_{i}_ISO{iso}_F{fnumber}_SS{speed}_{ext_light_intensity}')
        else:
            set_and_capture(speed, f'{fileprefix}_{i}_ISO{iso}_F{fnumber}_SS{speed}')

def test () -> None:
    parser = argparse.ArgumentParser(description='Capture image with gphoto2')
    parser.add_argument('--shutter-speed', type=int, default=1, help='Shutter speed')
    parser.add_argument('--filename', type=str, default='temp', help='Filename', required=False)
    args = parser.parse_args()

    # clean_connection()
    # set_shutter_speed(args.shutter_speed)
    # capture(args.filename)

    # print(f"ISO: {get_iso()}")
    # fnumber = get_fnumber()
    # if fnumber != -1:
    #     print(f"f-number: {fnumber}")
    # clean_connection()

def main():
    # half_stops = [15,20,30,45,60,90,128,180,250,350,500,750,1000]
    # full_stops = [4,8,15,30,60,125,250,500,1000,2000]
    full_stops = [8000, 6400, 5000, 4000, 3200, 2500, 2000, 1600, 1250, 1000, 800, 640, 500, 400, 320, 250, 200, 160, 125, 100, 80, 60, 50, 40, 30, 25, 20, 15, 13, 10, 8, 6, 5, 4, 3]
    print ("Running full stops of length", len(full_stops))
    light = 0
    # loop_capture(full_stops, 'le'+str(light)+'/test', 'ext_light_'+str(light))
    # loop_capture(full_stops, 'le'+str(light)+'/test')


if __name__ == '__main__':
    # test()
    # main()
    pass


