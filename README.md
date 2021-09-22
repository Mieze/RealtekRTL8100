RealtekRTL8100
==============

<H3>This project has been archived because all development activites have been stopped a long time ago and there won't be any updates in the future.

 Installation
------------

Before you install the driver you have to remove any installed driver for RTL810X.

1. Goto /S/L/E and delete the old driver.

2. Recreate the kernel cache.

3. Open System Preferences and delete the corresponding network interface, e. g. en0. If you forget this step you might experience strange problems with certain Apple domains, iTunes and iCloud later.

4. Reboot.

5. Install the new driver and recreate the kernel cache.

6. Reboot

7. Open System Preferences again, select Network and check if the new network interface has been created automatically or create it manually now.

8. Configure the interface.


Troubleshooting
---------------

- Make sure you have followed the installation instructions especially when you have issues with certain domains while the others are working fine.

- In Terminal type "kextstat" or "grep RealtekRTL8100 /var/log/system.log" to verify that the driver loads.
 
- Use the debug version to collect log data when trying to track down problems. The log messages can be found in /var/log/system.log. Include the log data when asking for support or giving feedback. I'm an engineer, not a clairvoyant.

- Check your BIOS settings. You might want to disable Network Boot and the UEFI Network Stack as these can interfere with the driver.

- Double check that you have removed any other Realtek kext from your system because they could prevent the driver from working properly.

- Verify your bootloader configuration, in particular the kernel flags. Avoid using npci=0x2000 or npci=0x3000.

- In Terminal run netstat -s in order to display network statistics. Carefully examine the data for any unusual activity like a high number of packets with bad IP header checksums, etc.

- In case auto-configuration of the link layer connection doesn't work it might be necessary to select the medium manually in System Preferences under Network for the interface.

- Use Wireshark to create a packet dump in order to collect diagnostic information.

- Keep in mind that there are many manufacturers of network equipment. Although Ethernet is an IEEE standard different implementations may show different behavior causing incompatibilities. In case you are having trouble try a different switch or a different cable or disable EEE.


Changelog
---------

Version 2.0.0 (2017-04-01):
 - Uses Apple's private driver interface introduced with 10.8.
 - Supports packet scheduling with QFQ.
 - Adds support for the RTL8107E.
 - Adds support for flow control and EEE.

Version 1.0.0 (2014-05-24):
- First offical release.
