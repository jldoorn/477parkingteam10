# 477parkingteam10

Need to install python 3.6.8 and then use requirements.txt to load dependencies through pip


## Folder Structure
```
.
└── 477team10/
    ├── esp_demo
    ├── uartlite/
    │   ├── Core/
    │   │   ├── Inc/
    │   │   │   └── main.h
    │   │   └── Src/
    │   │       └── main.c
    │   └── uartlite.ioc
    └── USSensor
```

### esp_demo
This folder contains python code that was used in the first prototype for our project, where the team connected two wifi modules to a laptop and successfully sent UDP packets between them.

### uartlite
This folder contains the integrated source code for the team's project, and should be opened in STM32CubeIDE for the best results. The `uartlite.ioc` file can be opened in STM32CubeMX to view the current pinout for the microcontroller for this project. All source code is stored in `uartlite/Core/Src`, and paired with header files stored in `uartlite/Core/Inc`.

### USSensor
This folder contains prototype source code for the proximity sensor used by this project. It can be opened in SystemWorkbench for STM32. The code in this folder has been integrated into the `uartlite` codebase, and is not being maintained.