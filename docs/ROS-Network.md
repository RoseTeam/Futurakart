# ROS network configuration

We use two Raspberry Pi 3 cards to run ROS and contol motors and sensors.

The first card RPi, called *base*, is responsible 









### Wifi network configuration

#### Create a hotspot on Raspberry using `nmcli`

```
sudo nmcli dev wifi hotspot ifname wlan0 con-name "futurakart_hs" ssid "futurakart_hs" password "FuturaKart Strong Password"
```

Static ip `10.42.1.1` to connect to the *base* RPi :

The ip `10.42.0.120` to connect to the *vision* RPi from *base* RPi
