# RPi and Mbed are the best friends



### Upload your firmware to `mbed`
  
This a particular case when you connect `mbed` cart to `rpi` and access 
to `rpi` using ssh or sftp.

Use SSH access to 
1) Mount `mbed` accessible point to `rpi`. Search it as `/dev/sd*`.
Make a mount point folder, e.g. `mkdir ~/mbed_repository`
```
sudo mount /dev/sda ~/mbed_repository -o gid=ubuntu,uid=ubuntu
```

To make a permanent mount see below. 

2) Connect to `rpi` from the desktop using `sftp`:
- open nautilus
- "Connect to server" and write `sftp://username@<IP>`
- Copy firmware file to `~/mbed_repository`


### Permanent mount `mbed` to `rpi`

Assuming that you are already mounted the `mbed` and created mount point 
at `~/mbed_repository`
 
Find out the UUID:
```
sudo blkid
...
/dev/sda: SEC_TYPE="msdos" LABEL="NUCLEO" UUID="2702-1974" TYPE="vfat" 
```

Add to the `/etc/fstab` the following line:
```
UUID="2702-1974" /home/ubuntu/mbed_repository vfat nofail,gid=ubuntu,uid=ubuntu,x-systemd.device-timeout=10 0 0
```
See [this](http://www.pclosmag.com/html/Issues/200709/page07.html) and [this](http://unix.stackexchange.com/questions/314271/automount-disk-connected-through-usb-when-the-pc-is-turned-on) for more info



