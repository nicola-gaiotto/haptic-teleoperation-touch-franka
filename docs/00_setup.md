# Setup

How the development machine was configured. The 3D Systems software is only needed for
Deliverable 1 and for the kinematic calibration (read-only); the Deliverable 2 driver
talks to the device directly over USB.

## Hardware and system

| Item | Version |
|---|---|
| Haptic device | 3D Systems Touch, USB (HID), USB ID `2988:0304`, external power supply |
| Computer | Laptop, Intel Core Ultra 7 258V, 32 GB RAM |
| OS | Ubuntu 24.04.5 LTS, kernel 7.0 (HWE) |
| ROS 2 | Jazzy (`/opt/ros/jazzy`) |
| 3D Systems driver | Touch Device Driver 2025.12.10 for Linux |
| SDK | OpenHaptics 3.4 Developer Edition for Linux |

Ubuntu 24.04 is required on this laptop (the Wi-Fi card is not supported by 22.04).

## 1. ROS 2 Jazzy

Follow the official installation guide for Ubuntu 24.04 (deb packages):
<https://docs.ros.org/en/jazzy/Installation.html>

## 2. 3D Systems software (Deliverable 1 and calibration only)

Download both packages from the official page
"OpenHaptics for Linux Developer Edition v3.4" on support.3dsystems.com.

**OpenHaptics 3.4** (installs to `/opt/OpenHaptics/Developer/3.4-0`, headers in
`/usr/include/HD*`, libraries `libHD`, `libHL` in `/usr/lib`):

```bash
tar xzf openhaptics_3.4-0-developer-edition-amd64.tar.gz
cd openhaptics_3.4-0-developer-edition-amd64
sudo ./install          # answer "y", then "q" to skip the automatic reboot
```

**Compatibility libraries** needed by OpenHaptics 3.4 on Ubuntu 24.04:

```bash
sudo apt install libglut3.12 libglut-dev
# libncurses5 and libtinfo5 are no longer in 24.04: install the 22.04 .deb packages
sudo dpkg -i libtinfo5_6.3-2ubuntu0.3_amd64.deb libncurses5_6.3-2ubuntu0.3_amd64.deb
# OpenHaptics looks for libglut.so.3
sudo ln -s /lib/x86_64-linux-gnu/libglut.so.3.12.0 /usr/lib/x86_64-linux-gnu/libglut.so.3
```

**Touch Device Driver 2025.12.10** (copies `libPhantomIOLib42.so` and
`libPhantomManagerLite.so` to `/usr/lib` and the udev rule `99-3dsystems.rules`):

```bash
tar xzf TouchDriver_2025_12_10+1.tgz
cd TouchDriver_2025_12_10
./install_haptic_driver
sudo reboot
```

Notes from the official page, applied here:
- for HID devices the folder `~/.3dsystems/config` should be empty before first use, and
  `Touch_HeadlessSetup` is **not** required;
- with a non-English OS, set the locale to English (`LANG`, `LC_NUMERIC`,
  e.g. Settings → Region & Language → Formats: United States).

Check with the diagnostic tool (run it from its own folder):

```bash
cd TouchDriver_2025_12_10/bin && ./TouchCheckup
```

## 3. USB access for the direct driver

The official udev rule already gives read/write access to the device:

```
SUBSYSTEMS=="usb", ATTRS{idVendor}=="2988", ATTRS{idProduct}=="0304", GROUP="users", MODE="0666"
```

Python USB library and numpy:

```bash
sudo apt install python3-usb python3-numpy
```

## 4. Workspace

```bash
mkdir -p ~/geomagic_ws/src && cd ~/geomagic_ws/src
git clone https://github.com/nicola-gaiotto/haptic-teleoperation-touch-franka.git
# Deliverable 1 driver (optional): see third_party/geomagic_touch_ros2/README.md
cd ~/geomagic_ws
colcon build --symlink-install
source install/setup.bash
```

Only one program at a time can use the Touch: close the Deliverable 1 driver before
starting `touch_raw_driver`, and vice versa.
