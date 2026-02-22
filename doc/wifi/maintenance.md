# HaHaHo access point
The RPI3 based HaHaHo access point connects to the publicroam WiFi and offers
simple SSID + password WiFi access to microcontrollers, laptops, tablets and phones.
This writeup gives some background useful if it stops working some day.

## Hardware inspection

- the RPi3 should have a solid 5V 4A power supply (the additional radio draws a lot of power)
- the radio1 USB slot should contain an Edimax EW-7811Un WiFi dongle. This slot sits bottom
  right with respect to the ethernet port.

## System log inspection

If the access point does not show a visible HaHaHo SSID, the most certain way to get access
is to connect it to a LAN with an ethernet cable. The RPI3 will receive an IP address on this
LAN and with this address one can access the LuCI web interface, which contains the system log.

On linux or wsl one can find the IP address on the LAN using:

sudo nmap -sL 192.168.2.0/24

Look for an output line:

Nmap scan report for OpenWrt.your.domain (192.168.2.123)

## Renewing the publicroam certificate

For authentication towards publicroam, OpenWRT needs a valid certificate. This certificate is
renewed every year and for now, we do the renewal on the RPi manually. Connect with putty/ssh
and use the following commands (this assumes that openssl-util are installed in OpenWRT via
the LuCI web interface):

cd /tmp
wget https://publicroam.nl/certificate-latest
openssl x509 -inform der -in pr-nl6.cer -out pr-nl6.pem
mv pr-nl6.pem /etc/ssl/certs/

Automating this would only be useful if publicroam uses overlapping validity periods for their
radius server certificates and if the HaHaHo access point is powered up every meeting.
