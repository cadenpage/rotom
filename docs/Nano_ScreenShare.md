# Jetson Orin Nano Headless GUI to macOS via Screen Sharing (VNC)

This guide documents the exact steps we used to stream a GPU-accelerated desktop from a headless Jetson Orin Nano to a Mac using macOS Screen Sharing (VNC). It keeps NVIDIA acceleration (Xorg + GLX), forces a virtual 1080p display via EDID, and makes the session available on port 5900.

## Prerequisites
- Jetson Orin Nano running JetPack 6.x (L4T 36.x)
- SSH access to the Jetson
- macOS client on the same network
- Admin rights on both machines

## Summary of What Works
- Xorg (not Wayland) with NVIDIA driver and GLX
- Forced virtual display at 1920×1080 using a Custom EDID
- macOS Screen Sharing connecting to Jetson’s VNC server
- Autologin to ensure an active desktop on `:0`
- Optional x11vnc service (TigerVNC also works, but Screen Sharing is smoother here)

## 1) Generate an EDID.bin on macOS from this repo
Pick any 1080p EDID text file. Example used below:

```zsh
cd /Users/cadenpage/Documents/EDID
cat "Digital/AOC/AOC0000/25DAFB6ACD4C" | grep -E '^([a-f0-9]{32}|[a-f0-9 ]{47})$' | tr -d '[:space:]' | xxd -r -p > EDID.bin
```

(Optional) Verify on Jetson later with `edid-decode`.

## 2) Copy EDID.bin to Jetson and place it in firmware
```zsh
# From macOS
scp /Users/cadenpage/Documents/EDID/EDID.bin <jetson-user>@<jetson-host-or-ip>:/tmp

# On Jetson
sudo mkdir -p /lib/firmware/edid
sudo cp /tmp/EDID.bin /lib/firmware/edid/EDID.bin
```

## 3) Ensure Xorg (not Wayland) and enable autologin
Edit GDM config:
```zsh
sudo nano /etc/gdm3/custom.conf
```
Under `[daemon]`:
```
[daemon]
WaylandEnable=false
AutomaticLoginEnable=true
AutomaticLogin=<your_username>
```

Set the graphical target:
```zsh
systemctl get-default
sudo systemctl set-default graphical.target
```

## 4) Enable NVIDIA DRM KMS (required for NVIDIA Xorg)
Edit `/boot/extlinux/extlinux.conf` and add `nvidia-drm.modeset=1` to the active boot entry (`LABEL JetsonIO` or `primary`). Example:
```
APPEND ${cbootargs} ... console=tty0 nvidia-drm.modeset=1
```
Apply and reboot:
```zsh
sudo update-initramfs -u
sudo reboot
```
Confirm:
```zsh
cat /proc/cmdline | grep nvidia-drm.modeset=1
```

## 5) Force a fake monitor via Xorg with Custom EDID
Use a minimal Xorg config (DFP-1 was correct for Orin Nano HDMI/TMDS):
```zsh
sudo mkdir -p /etc/X11/xorg.conf.d
sudo nano /etc/X11/xorg.conf.d/10-nvidia-edid.conf
```
Paste:
```
Section "Device"
    Identifier  "Tegra0"
    Driver      "nvidia"
    Option      "AllowEmptyInitialConfiguration" "true"
    Option      "ConnectedMonitor" "DFP-1"
    Option      "CustomEDID" "DFP-1:/lib/firmware/edid/EDID.bin"
    Option      "UseDisplayDevice" "DFP-1"
    Option      "IgnoreEDIDChecksum" "DFP-1"
    Option      "AllowNonEdidModes" "true"
EndSection

Section "Screen"
    Identifier "Screen0"
    Device     "Tegra0"
    DefaultDepth 24
    SubSection "Display"
        Depth 24
        Modes "1920x1080"
    EndSubSection
EndSection
```

Restart GDM or reboot:
```zsh
sudo systemctl restart gdm
# or
sudo reboot
```

Verify 1080p at `:0`:
```zsh
DISPLAY=:0 xrandr
# Expect: DP-1 connected primary 1920x1080
```

## 6) Confirm NVIDIA acceleration
```zsh
sudo apt-get update && sudo apt-get install -y mesa-utils
DISPLAY=:0 xhost +local:
DISPLAY=:0 glxinfo | grep -E "OpenGL renderer|version"
# Expect renderer: NVIDIA Tegra Orin (nvgpu)/integrated and OpenGL 4.6 NVIDIA
```

## 7) Start a VNC server (optional service)
For Screen Sharing, you can use either x11vnc or TigerVNC.
We used x11vnc with minimal stable flags:
```zsh
# Create password for the autologin user
sudo -u <your_username> x11vnc -storepasswd

# One-off run (test)
x11vnc -usepw -display :0 -forever -rfbport 5900 -listen 0.0.0.0 -shared
```

To run at boot via systemd:
```zsh
sudo nano /etc/systemd/system/x11vnc.service
```
Paste:
```
[Unit]
Description=Start x11vnc at startup
Requires=display-manager.service
After=display-manager.service

[Service]
Type=simple
User=<your_username>
ExecStart=/usr/bin/x11vnc -auth guess -display WAIT:0 -usepw -forever -rfbport 5900 -listen 0.0.0.0 -shared
Restart=on-failure

[Install]
WantedBy=graphical.target
```
Apply:
```zsh
sudo systemctl daemon-reload
sudo systemctl enable x11vnc.service
sudo systemctl restart x11vnc.service
systemctl status x11vnc.service
ss -ltnp | grep 5900
```

## 8) Connect from macOS (Screen Sharing)
Use Apple’s Screen Sharing app with the VNC URL:
```zsh
open -a "Screen Sharing" vnc://<jetson-ip>:5900
```
Tips:
- In the viewer menu: View → Turn Scaling On, Window → Enter Full Screen.
- If TigerVNC is used instead, avoid server resize requests:
```zsh
vncviewer <jetson-ip>:5900 -AutoSelect=0 -RemoteResize=0
```

## 9) Make the UI more comfortable
- GNOME text scaling:
```zsh
gsettings set org.gnome.desktop.interface text-scaling-factor 1.25
# or 1.50
```
- Disable animations (reduces overhead):
```zsh
gsettings set org.gnome.desktop.interface enable-animations false
```

## 10) Optional performance boost (Jetson)
```zsh
sudo nvpmodel -m 0
sudo jetson_clocks
```
Use wired Ethernet for best responsiveness.

## Troubleshooting
- If `DISPLAY=:0 xrandr` shows 640×480 or “disconnected”, check:
  - `/etc/gdm3/custom.conf` has `WaylandEnable=false` and autologin enabled.
  - `/boot/extlinux/extlinux.conf` includes `nvidia-drm.modeset=1` in the active boot entry.
  - The Xorg log `/var/log/Xorg.0.log` shows NVIDIA driver + your `CustomEDID` and correct connector (`DFP-1` or `DFP-0`).
- If macOS says “Make sure Screen Sharing … is enabled”: use the VNC URL `vnc://<jetson-ip>:5900` (it connects to x11vnc), not Apple Remote Management.
- If VNC connection is refused: verify x11vnc is listening with `ss -ltnp | grep 5900` and check service logs `journalctl -u x11vnc.service`.

## Verification Snippets
- Xorg NVIDIA + GLX present:
```
(II) Module nvidia: vendor="NVIDIA Corporation"
(II) NVIDIA GLX Module 540.4.0
(II) NVIDIA(0): Validated MetaModes: "DFP-1:1920x1080"
```
- GLX renderer:
```
OpenGL renderer string: NVIDIA Tegra Orin (nvgpu)/integrated
OpenGL version string: 4.6.0 NVIDIA 540.4.0
```

## Notes
- Connector naming: NVIDIA Xorg uses `DFP-0/DFP-1`. Kernel DRM names show as `DP-0/DP-1`. For `CustomEDID`, use the NVIDIA names.
- EDID source: this repository provides decoded EDID texts; convert them to binary with the one-liner above.
- Security: Autologin is convenient for development but should be disabled in production.
