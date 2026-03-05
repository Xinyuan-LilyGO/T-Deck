//                            USER DEFINED SETTINGS
//   Set driver type, fonts to be loaded, pins used and SPI control method etc.

#define USER_SETUP_INFO "T-Deck ST7789"

// ##################################################################################
// Section 1. Driver and Display Configuration
// ##################################################################################

#define ST7789_DRIVER      // ST7789 driver for T-Deck
#define TFT_RGB_ORDER TFT_RGB  // Colour order Red-Green-Blue

// Display resolution (T-Deck is 320x240 in portrait)
#define TFT_WIDTH  320
#define TFT_HEIGHT 240

// ##################################################################################
// Section 2. Pin Configuration for ESP32-S3
// ##################################################################################

// SPI pins for T-Deck
#define TFT_MOSI  41    // SPI MOSI
#define TFT_SCLK  40    // SPI Clock
#define TFT_CS    12    // Chip Select
#define TFT_DC    11    // Data/Command
#define TFT_RST   -1    // Reset (tied to EN pin, set to -1)
#define TFT_BL    42    // Backlight control pin

// ##################################################################################
// Section 3. SPI Frequency Configuration
// ##################################################################################

#define SPI_FREQUENCY  40000000  // 40 MHz for ST7789
#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY  2500000

// Use HSPI port for ESP32-S3
#define USE_HSPI_PORT

// ##################################################################################
// Section 4. Font Configuration
// ##################################################################################

#define LOAD_GLCD   // Original Adafruit 8 pixel font
#define LOAD_FONT2  // Small 16 pixel font
#define LOAD_FONT4  // Medium 26 pixel font
#define LOAD_FONT6  // Large 48 pixel font
#define LOAD_FONT7  // 7 segment 48 pixel font
#define LOAD_FONT8  // Large 75 pixel font
#define LOAD_GFXFF  // FreeFonts (48 Adafruit_GFX fonts)
#define SMOOTH_FONT // Smooth anti-aliased fonts

// ##################################################################################
// Section 5. Other Options
// ##################################################################################

// Uncomment if colors appear inverted
// #define TFT_INVERSION_ON

// Support for SPI transactions (needed for multiple SPI devices)
// Automatically enabled for ESP32, but you can explicitly set it
// #define SUPPORT_TRANSACTIONS
