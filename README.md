# IGVC25 Capstone Software
> subtitle!
---

To use this software you first need an ubuntu 22 environment:

1.  Ubuntu computer
2.  [WSL](https://learn.microsoft.com/en-us/windows/wsl/install)
3.  [VM](https://www.virtualbox.org)

Once you are in, I reccomend setting up [ssh with git.](https://docs.github.com/en/authentication/connecting-to-github-with-ssh) This will make pushing and pulling eaiser.

Either way, clone this repo, and navigate to it via terminal:

```bash
git clone git@github.com:e-spinner/IGVC25.git && cd IGVC25
```

### Installing Ros

The following script will install ros-humble and the required packages for this repo + build the software:

```bash
./setup.sh
```

> If this raises a permision denied error. run `chmod +x ./setup.sh` and try again.

### Setting up Sensors

If you need to use the Sensors, run the following command to setup udev rules that the software relies on to identify which usb port devices are in:

```bash
sudo cp 99-usb-sensors.rules /etc/udev/rules.d/99-usb-sensors.rules
```

now reload the udev rules:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```