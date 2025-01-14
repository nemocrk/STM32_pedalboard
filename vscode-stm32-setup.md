# Setting up VSCode for STM32 Development

## PART 1: Required Software Installation

### 1. Install Base Software
1. Install [Visual Studio Code](https://code.visualstudio.com/)
2. Install [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)
3. Install [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
4. Install [MinGW](https://sourceforge.net/projects/mingw/) (Windows) or GCC (Linux/Mac)
5. Install [Make](https://gnuwin32.sourceforge.net/packages/make.htm) (Windows only, included in Linux/Mac)
6. Install [OpenOCD](https://openocd.org/pages/getting-openocd.html)

### 2. Install VSCode Extensions
1. Open VSCode
2. Install these extensions:
   - C/C++ Extension Pack
   - Cortex-Debug
   - STM32 VS Code Extension

### 3. Add Tools to System PATH
Add these to your system's PATH environment variable:
- arm-none-eabi-gcc bin directory
- OpenOCD bin directory
- Make binary directory
- STM32CubeProgrammer binary directory

## PART 2: Project Setup

### 1. Create Project with STM32CubeMX
1. Open STM32CubeMX
2. Create new project
3. Select STM32H723ZGT6
4. Configure your pins and peripherals
5. Go to Project Manager
6. Set Toolchain/IDE to "Makefile"
7. Generate code

### 2. Setup VSCode Project
1. Open the generated project folder in VSCode
2. Create a `.vscode` folder
3. Create these configuration files:

`c_cpp_properties.json`:
```json
{
    "configurations": [
        {
            "name": "STM32",
            "includePath": [
                "${workspaceFolder}/**",
                "Core/Inc/**",
                "Drivers/STM32H7xx_HAL_Driver/Inc/**",
                "Drivers/STM32H7xx_HAL_Driver/Inc/Legacy/**",
                "Drivers/CMSIS/Device/ST/STM32H7xx/Include/**",
                "Drivers/CMSIS/Include/**"
            ],
            "defines": [
                "USE_HAL_DRIVER",
                "STM32H723xx"
            ],
            "compilerPath": "arm-none-eabi-gcc",
            "cStandard": "c11",
            "cppStandard": "c++14",
            "intelliSenseMode": "gcc-arm"
        }
    ],
    "version": 4
}
```

`launch.json`:
```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Debug STM32",
            "cwd": "${workspaceFolder}",
            "executable": "./build/PCM3168A_Project.elf",
            "request": "launch",
            "type": "cortex-debug",
            "servertype": "openocd",
            "device": "STM32H723ZG",
            "configFiles": [
                "interface/stlink.cfg",
                "target/stm32h7x.cfg"
            ],
            "svdFile": "${workspaceFolder}/STM32H723.svd"
        }
    ]
}
```

`tasks.json`:
```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "Build",
            "type": "shell",
            "command": "make",
            "group": {
                "kind": "build",
                "isDefault": true
            }
        },
        {
            "label": "Clean",
            "type": "shell",
            "command": "make clean"
        }
    ]
}
```

## PART 3: Using VSCode

### 1. Building the Project
- Press `Ctrl+Shift+B` to build
- Or use Terminal → Run Build Task
- Output appears in Terminal window

### 2. Debugging
1. Connect your ST-Link debugger
2. Click the Debug icon in the sidebar
3. Click the play button to start debugging
4. Use standard debug controls:
   - F5: Continue
   - F10: Step Over
   - F11: Step Into
   - Shift+F5: Stop

### 3. Useful VSCode Shortcuts
- `Ctrl+P`: Quick file open
- `F12`: Go to definition
- `Alt+←`: Go back
- `Ctrl+Space`: Trigger suggestions
- `Ctrl+Shift+P`: Command palette

## PART 4: Adding PCM3168A Code

### 1. Create Project Files
Create these files in your project:
```
src/
  ├── pcm3168a_dsp.c
  ├── pcm3168a_dsp.h
  ├── pcm3168a_driver.c
  └── pcm3168a_driver.h
```

### 2. Update Makefile
Add your new files to the Makefile:
```makefile
C_SOURCES += \
src/pcm3168a_dsp.c \
src/pcm3168a_driver.c
```

## PART 5: Useful Tips

### 1. Code Navigation
- Right-click → Go to Definition
- Right-click → Find All References
- Use the Outline view for file structure

### 2. Building
- Build: `make`
- Clean: `make clean`
- Rebuild: `make clean all`

### 3. Flashing
Use STM32CubeProgrammer or OpenOCD:
```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program build/your_project.elf verify reset exit"
```

### 4. Debugging Tips
- Use Watch window for variables
- Set breakpoints by clicking left of line numbers
- Use Memory view for register inspection

## Common Issues and Solutions

1. "arm-none-eabi-gcc not found":
   - Check PATH environment variable
   - Reinstall ARM toolchain

2. "Make not found":
   - Check PATH environment variable
   - Reinstall Make

3. Debug not working:
   - Check ST-Link connection
   - Verify OpenOCD installation
   - Check debug configuration in launch.json

4. IntelliSense not working:
   - Reload VSCode
   - Check c_cpp_properties.json
   - Verify include paths
