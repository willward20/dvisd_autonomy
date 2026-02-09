# Setting Up a New Raspberry Pi

## Enable SSH Connection over `utexas-iot`

### Find the Pi's MAC Address
The MAC address looks like `a1:b2:c3:d4:e5:f6`. Run this command:
```
ip link show
```
Look for this line under `wlan0`:
```
link/ether a1:b2:c3:d4:e5:f6 brd ff:ff:ff:ff:ff:ff
```

### Register the Pi on `utexas-iot`
1. Go to [network.utexas.edu](https://network.utexas.edu/), login with a UT EID, and "Register Wi-Fi Device".
2. Enter the MAC address. Name the device. Set `Network Profile` to `Unprotected`.

### Connect the Pi to `utexas-iot`
1. Go to Wi-Fi settings and select `utexas-iot`
2. Enter the password for this device found on the [network.utexas.edu](https://network.utexas.edu/) page (exclude spaces). 

---

## Configure System Time

Check that 
```
date
```
outputs the correct date and time. If not, the run the following commands and check the output of `date` again.
```
sudo timedatectl set-ntp true
sudo systemctl restart systemd-timesyncd
```
