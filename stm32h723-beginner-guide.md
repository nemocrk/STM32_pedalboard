# Beginner's Guide: STM32H723 + PCM3168A Setup

## PART 1: Software Installation

### Step 1: Install STM32CubeIDE
1. Go to [ST's website](https://www.st.com/en/development-tools/stm32cubeide.html)
2. Click "Get Software" 
3. Download the version for your operating system (Windows/Mac/Linux)
4. Run the installer
5. Accept all default options during installation

### Step 2: Install STM32 Packages
1. Open STM32CubeIDE
2. Go to Help → Update Software
3. Wait for the package list to load
4. Find and install "STM32H7xx" package

## PART 2: Create Your Project

### Step 1: Start a New Project
1. Click File → New → STM32 Project
2. In the search box, type "STM32H723ZG"
3. Select your board
4. Click "Next"
5. Name your project (example: "PCM3168A_Project")
6. Click "Finish"

### Step 2: Basic Project Setup
Your project opens with a .ioc file. This is where we'll set up the microcontroller.

1. Set up the Clock:
   - Click "Clock Configuration" tab
   - Set HCLK to 550 MHz (this is your main clock)
   - The tool will automatically calculate other clocks

2. Set up the Pins:
   - Click "Pinout & Configuration" tab
   - We'll need these peripherals:
     - SAI2 (for audio)
     - I2C4 (for controlling the codec)
     - A few GPIO pins

## PART 3: Configure Audio Interface

### Step 1: Set up SAI2
1. In the .ioc file, find "SAI2" in the left panel
2. Enable SAI2 by clicking the checkbox
3. Configure SAI2:
   - Mode: Master Transmitter
   - Protocol: I2S
   - Data Size: 24 bits
   - Audio Frequency: 48000 Hz

### Step 2: Set up Control Pins
1. Find these pins in the pinout view:
   - PE2: Set as GPIO_Output (Reset pin)
   - PE3: Set as GPIO_Output (Mode pin)

### Step 3: Generate Code
1. Click the "Generate Code" button (looks like a gear icon)
2. Wait for code generation to complete
3. When asked to open the main.c file, click "Yes"

## PART 4: Add Basic Code

### Step 1: Add Headers
Add these at the top of main.c:
```c
#include "main.h"
#include "pcm3168a_dsp.h"

// Buffer for audio data
#define BUFFER_SIZE 256
int32_t AudioBufferIn[BUFFER_SIZE];
int32_t AudioBufferOut[BUFFER_SIZE];
```

### Step 2: Initialize Hardware
Add this code in main():
```c
int main(void)
{
    // Initialize system
    HAL_Init();
    SystemClock_Config();

    // Initialize GPIO
    MX_GPIO_Init();
    
    // Initialize audio interface
    MX_SAI2_Init();
    
    // Reset PCM3168A
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
    
    while (1)
    {
        // Main loop
        HAL_Delay(100);
    }
}
```

## PART 5: Basic Testing

### Step 1: Test Hardware Connection
1. Connect your PCM3168A to the STM32 board:
   ```
   STM32H723    PCM3168A
   PC6    -->   MCLK
   PD13   -->   BCLK
   PD11   -->   SDIN
   PD12   -->   LRCK
   PE2    -->   RESET
   ```

2. Connect power:
   ```
   3.3V   -->   DVDD
   3.3V   -->   AVDD
   GND    -->   DGND
   GND    -->   AGND
   ```

### Step 2: Verify Setup
1. Compile your project (click the hammer icon)
2. If there are errors, they'll show in the "Problems" view
3. Fix any errors before continuing

## PART 6: Add Audio Processing

### Step 1: Add Basic Audio Processing
Add this function to process audio:
```c
void ProcessAudio(int32_t* input, int32_t* output, uint32_t size)
{
    // Simple passthrough for testing
    for(uint32_t i = 0; i < size; i++)
    {
        output[i] = input[i];
    }
}
```

### Step 2: Add DMA Callback
Add this function to handle audio data:
```c
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
    // Process audio when DMA buffer is full
    ProcessAudio(AudioBufferIn, AudioBufferOut, BUFFER_SIZE);
}
```

## Next Steps

After completing this basic setup, you can:
1. Test with a simple audio signal
2. Add volume control
3. Add basic filters
4. Add more complex DSP features

Common Problems and Solutions:
1. No compilation: Make sure all files are included in project
2. No audio: Check your wire connections
3. Noise in audio: Check your ground connections
4. System not responding: Check clock settings

## Need Help?
- Check ST's forums for common issues
- Review your connections
- Verify clock settings
- Use an oscilloscope to check signals if available
