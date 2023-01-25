# ESP Demo

This folder contains source code to demonstrate tx and rx functions of the ESP wifi module

## Module Demo

With an ESP-01 module plugged into a computer via an FTDI adapter, run
`python test_esp_rx.py <serial_file>`

This will setup the ESP-01 module to be an accesspoint with ssid `myapteam10` and password `012345678`

Connect your laptop to this wifi network. Then run 
`python test_tx.py 192.168.0.1 8080` in another terminal

You should see reports of cars entering and exiting in the terminal that is running the receive script