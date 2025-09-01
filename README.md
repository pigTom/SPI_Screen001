# STM32 ST7735S TFT Display Driver

A comprehensive STM32 project for driving ST7735S TFT displays with performance testing and graphics capabilities.

## 📋 Project Overview

This project demonstrates how to interface an STM32F103 microcontroller with an ST7735S TFT display using SPI communication. It includes a complete display driver library with graphics functions, font rendering, and performance testing capabilities.

## 🔧 Hardware Configuration

### Display Connections
- **SPI1 Interface:**
  - SCK: PA5 (DISPL_SCK)
  - MOSI: PA7 (DISPL_MOSI)
  - CS: PA4 (DISPL_CS)
- **Control Pins:**
  - RST: PB0 (DISPL_RST) - Reset pin
  - DC: PB1 (DISPL_DC) - Data/Command select
  - BLK: PC13 (DISPL_LED) - Backlight control

### Display Specifications
- **Resolution:** 160x128 pixels (configurable for other sizes)
- **Color Format:** RGB565 (16-bit color)
- **Interface:** SPI
- **Controller:** ST7735S

## 🚀 Features

### Core Display Functions
- Display initialization and configuration
- Pixel-level drawing operations
- Area filling with solid colors
- Line drawing (horizontal, vertical, diagonal)
- Rectangle and border drawing
- Image rendering support

### Text Rendering
- Multiple font sizes support
- String rendering with background
- Centered text alignment
- Custom font integration

### Performance Testing
- `Displ_PerfTest()` - Comprehensive performance benchmarking
- Screen fill tests
- Line drawing performance
- Graphics rendering speed tests

### Backlight Control
- Simple on/off control (default)
- PWM dimming support (configurable)
- Standby mode support

## 📁 Project Structure

```
SPI_Screen001/
├── Core/
│   ├── Inc/
│   │   ├── main.h              # Main header with pin definitions
│   │   ├── z_displ_ST7735.h    # Display driver header
│   │   └── fonts.h             # Font definitions
│   └── Src/
│       ├── main.c              # Main application
│       ├── z_displ_ST7735.c    # Display driver implementation
│       └── z_displ_ST7735_test.c # Performance tests
├── Drivers/                    # STM32 HAL drivers
└── README.md                   # This file
```

## 🛠️ Configuration

### SPI Configuration
- **Mode:** Master
- **Data Size:** 8-bit
- **Clock Polarity:** Low
- **Clock Phase:** 1st Edge
- **Baud Rate:** System Clock / 32
- **Direction:** Simplex (TX only)

### Display Size Configuration
Edit `z_displ_ST7735.h` to configure display size:
```c
#define ST7735_IS_160X128 1     // Enable for 160x128 displays
#define ST7735_WIDTH  128
#define ST7735_HEIGHT 160
```

### Backlight Configuration
For simple on/off control (default):
```c
// DISPLAY_DIMMING_MODE is commented out
```

For PWM dimming:
```c
#define DISPLAY_DIMMING_MODE    // Uncomment to enable
#define BKLIT_TIMER TIM3        // Timer for PWM
#define BKLIT_CHANNEL TIM_CHANNEL_2
```

## 🔨 Building and Running

### Prerequisites
- STM32CubeIDE or compatible toolchain
- STM32F103 development board
- ST7735S TFT display module

### Build Steps
1. Open the project in STM32CubeIDE
2. Configure your target STM32F103 variant
3. Build the project
4. Flash to your microcontroller

### Initialization Sequence
```c
// In main() function
Displ_Init(Displ_Orientat_0);    // Initialize display
Displ_CLS(BLACK);                // Clear screen
Displ_BackLight('I');            // Initialize and turn on backlight
```

## 📊 Performance Testing

The project includes comprehensive performance tests accessible via `Displ_PerfTest()`:

- **Test 1:** Complete graphics test suite
- **Test 2:** Screen fill performance (50 screens)
- **Test 3:** Line drawing performance (30k lines)
- **Test 4:** Rectangle drawing performance
- **Test 5:** Text rendering performance

## 🎨 Graphics API

### Basic Drawing
```c
Displ_Pixel(x, y, color);                    // Draw single pixel
Displ_Line(x0, y0, x1, y1, color);          // Draw line
Displ_FillArea(x, y, w, h, color);          // Fill rectangle
Displ_Border(x, y, w, h, thickness, color); // Draw border
```

### Text Rendering
```c
Displ_WString(x, y, "Hello", font, size, color, bgcolor);
Displ_CString(x, y, w, h, "Centered", font, size, color, bgcolor);
```

### Display Control
```c
Displ_CLS(color);                    // Clear screen
Displ_Orientation(orientation);      // Set display orientation
Displ_BackLight('1');               // Turn on backlight
Displ_BackLight('0');               // Turn off backlight
```

## 🐛 Troubleshooting

### Display Not Working
1. **Check backlight:** Ensure backlight is enabled after initialization
2. **Verify connections:** Double-check SPI and control pin connections
3. **Power supply:** Ensure adequate power supply for display
4. **SPI timing:** Verify SPI clock frequency is within display specifications

### Common Issues
- **Blank screen:** Usually backlight control issue - check if `HAL_GPIO_WritePin(DISPL_LED_GPIO_Port, DISPL_LED_Pin, GPIO_PIN_RESET)` is accidentally turning off backlight after initialization
- **Garbled display:** Check SPI configuration and timing
- **Wrong colors:** Verify RGB565 color format usage
- **Display orientation issues:** Use `Displ_Orientation()` to set correct orientation

### Known Issues Fixed
- **Backlight Control Conflict:** The original code had a bug where backlight was turned off immediately after initialization. This has been fixed by commenting out the conflicting GPIO write operation.

## 🔧 Hardware Notes

### Backlight Circuit
- The BLK pin is connected directly to PC13 without external current limiting resistor
- STM32F103 GPIO can typically source/sink up to 25mA per pin
- For high-power backlights, consider adding a transistor driver circuit

### SPI Timing
- Current configuration uses SPI_BAUDRATEPRESCALER_32
- For 72MHz system clock: SPI clock = 72MHz/32 = 2.25MHz
- ST7735S supports up to 15MHz SPI clock

## 📈 Performance Characteristics

Based on the performance test results:
- Screen fill operations: Optimized for bulk data transfer
- Line drawing: Efficient pixel-by-pixel operations
- Text rendering: Font-based with background support
- SPI communication: DMA support available for large transfers

## 🔄 Communication Modes

The driver supports multiple SPI communication modes:
- **Polling Mode** (default): Simple blocking transfers
- **Interrupt Mode**: Non-blocking with interrupt callbacks
- **DMA Mode**: High-speed transfers for large data blocks

Configure in `z_displ_ST7735.h`:
```c
// #define DISPLAY_SPI_INTERRUPT_MODE  // Uncomment for interrupt mode
// #define DISPLAY_SPI_DMA_MODE        // Uncomment for DMA mode
```

## 📝 License

This project is based on the ST7735S-STM32 library by maudeve-it.
Original library: https://github.com/maudeve-it/ST7735S-STM32

Please refer to the original licensing terms for usage and distribution.

## 🤝 Contributing

Contributions are welcome! Areas for improvement:
- Support for additional display sizes
- Enhanced graphics primitives
- Touch screen integration
- Power management features

## 📚 References

- [ST7735S Datasheet](https://www.displayfuture.com/Display/datasheet/controller/ST7735.pdf)
- [STM32F103 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [Original ST7735S Library](https://github.com/maudeve-it/ST7735S-STM32)
- [STM32 HAL Documentation](https://www.st.com/resource/en/user_manual/um1725-description-of-stm32f4-hal-and-lowlayer-drivers-stmicroelectronics.pdf)

---

**Note:** This project includes the fix for the backlight control issue where the display appeared blank due to conflicting GPIO operations after initialization.