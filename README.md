## Introduction                                                                                                     
This toolset helps to split serial communication interface into 2 ACM ports. 

### Prepating and Flashing
- To prepare *.bin or *.elf use next command
`make SERIALNUM-{ID}`

- Connect the programmator pins to you MC according to the datasheet or silkprint

- Upload the firmware to the controller 
flash write {PATH} 0x8000000

- If everything works correctly, you can connect you MC to USB port and check if your Serial port with the setted ID
```
$ ls -la /dev/serial/by-id/usb-*
 
 /dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_{ID}-if00 -> ../../ttyACM0
 /dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_{ID}-if02 -> ../../ttyACM1
```

###for Linux
- You should install first an open source tools from the [(Link)](https://github.com/stlink-org/stlink) and all the requirements for it
