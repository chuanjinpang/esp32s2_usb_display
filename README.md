# esp32s2_usb_display

**overview**

it's a USB mini display for Linux platform, such as raspberry Pi, Centos X86 server.
it refer many opensource projects: robopeak/rpusbdisp,Bodmer/TFT_eSPI.git ,nopnop2002/esp-idf-ili9340,serge-rgb/TinyJPEG.git,TJpgDec. thanks 
esp32s2 support USB OTG, the Linux host compress framebuffer Zone with JPEG, and then issue URB to esp32s2, the S2  wil decode JPEG stream bytes to RGB data,and use DMA SPI to ili9341 screen.

now it can run ~13pfs in most time.

**folder intro:**

/esp32s2_usbdisp    is esp32s2 esp-idf project, it will generate esp32s2 binary file for  programe

/rpusbdisp  is linux fb kernel driver module, it will generate rp_usbdisplay.ko

eps32s2_usb_display_readme.doc is a file for how to Hardware connect and other info in CN

**install:**

in rpusbdisp, it have a  README file which introduction how to install the ko and config xserver.

**Demo**

please seem below links for view demo:

![image](https://github.com/chuanjinpang/esp32s2_usb_display/blob/main/demo/u1.jpg)
![image](https://github.com/chuanjinpang/esp32s2_usb_display/blob/main/demo/u2.jpg)
![image](https://github.com/chuanjinpang/esp32s2_usb_display/blob/main/demo/u3.jpg)
![image](https://github.com/chuanjinpang/esp32s2_usb_display/blob/main/demo/u4.jpg)


https://www.bilibili.com/video/BV17L411s7kL?spm_id_from=333.999.0.0

https://www.bilibili.com/video/BV1dQ4y1z75s?spm_id_from=333.999.0.0

