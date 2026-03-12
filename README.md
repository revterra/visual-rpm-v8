# Visual RPM v8

High-speed RPM meter using a FLIR Blackfly S GigE camera (~280fps) with Aravis. Detects black tape on a rotating shaft via brightness thresholding to calculate RPM.

## Hardware

- **Camera:** FLIR Blackfly S BFS-PGE-04S2C (GigE PoE, Sony IMX287, 720x540, up to 291fps)
- **Connection:** Direct GigE Ethernet (no switch required), PoE or PoE injector for power

## Dependencies

```bash
# Aravis (GigE Vision / USB3 Vision library)
sudo apt install gir1.2-aravis-0.8 libaravis-0.8-0 aravis-tools

# Python packages
pip install numpy opencv-python

# GObject introspection (usually already installed)
sudo apt install python3-gi
```

## Network Setup

The GigE camera uses a link-local IP (169.254.x.x). Your Ethernet adapter needs an IP on the same subnet.

### 1. Add a link-local IP to your Ethernet adapter

```bash
sudo ip addr add 169.254.172.1/24 dev eno2
```

Replace `eno2` with your Ethernet interface name and `169.254.172.1` with an IP on the same /24 subnet as your camera.

### 2. Enable jumbo frames (MTU 9000)

Required to unlock frame rates above 200fps:

```bash
sudo ip link set eno2 mtu 9000
```

### 3. Increase network receive buffers

Without this, packets drop at high frame rates:

```bash
sudo sysctl -w net.core.rmem_max=16777216
sudo sysctl -w net.core.rmem_default=16777216
sudo sysctl -w net.core.netdev_max_backlog=10000
```

### Making settings persistent across reboots

The above commands don't survive a reboot. To make them permanent:

**For sysctl settings**, add to `/etc/sysctl.d/99-gige-camera.conf`:
```
net.core.rmem_max=16777216
net.core.rmem_default=16777216
net.core.netdev_max_backlog=10000
```

**For the IP address and MTU**, use NetworkManager or add to `/etc/network/interfaces`:
```
auto eno2
iface eno2 inet static
    address 169.254.172.1
    netmask 255.255.255.0
    mtu 9000
```

## Camera Configuration Notes

The script automatically configures the camera for maximum performance:

- **GainAuto = Off** — This is critical. `GainAuto: Continuous` (the default on this color camera) caps the frame rate at 200fps even though the hardware supports 291fps.
- **PixelFormat = Mono8** — Color is not needed for RPM detection; mono reduces bandwidth.
- **GevSCPSPacketSize = 9000** — Jumbo frames for efficient GigE transfer.
- **GevSCPD = 0** — Zero inter-packet delay for maximum throughput.
- **AcquisitionFrameRateEnable = False** — Free-run mode at max sensor speed.
- **32MB socket buffer** with 30 stream buffers — enough for throughput without adding lag.

## Usage

```bash
python3 rpm_meter_gui.py
```

With a custom camera IP:
```bash
python3 rpm_meter_gui.py --camera 169.254.172.226
```

### Controls

| Key | Action |
|-----|--------|
| Q | Quit |
| R | Toggle video recording (full fps) |
| W/A/S/D | Move ROI up/left/down/right |
| +/- | Resize ROI |
| Threshold trackbar | Adjust dark/light detection threshold |
| Exposure trackbar | Adjust camera exposure time |

### Recording

Press **R** to start/stop recording. Videos are saved as `.avi` (MJPG) in the script directory at the full camera frame rate (~280fps). Recordings play back in slow motion in most video players since they exceed 60fps.

## Finding Your Camera

To discover GigE cameras on your network:

```bash
arv-tool-0.8
```

Or in Python:
```python
import gi
gi.require_version('Aravis', '0.8')
from gi.repository import Aravis
Aravis.update_device_list()
for i in range(Aravis.get_n_devices()):
    print(Aravis.get_device_id(i), Aravis.get_device_address(i))
```
