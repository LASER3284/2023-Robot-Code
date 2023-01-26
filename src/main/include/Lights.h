#pragma once

#include <frc/AddressableLED.h>
#include <frc/util/Color.h>

namespace lights {
    class Constants {
        public:
            /// @brief The total LED count of the LED strip chain
            static constexpr int kStringLength = 28;
    };

    class LightHandler {
        public:
            LightHandler() {
                ledStrip.SetLength(Constants::kStringLength);
                ledStrip.SetData(ledBuffer);
                ledStrip.Start();
            }

            /// @brief Set the value of the LED pixel at the given index
            /// @param r Red (between 0 and 255)
            /// @param g Green (between 0 and 255)
            /// @param b Blue (between 0 and 255)
            /// @param index 
            void SetColor(int r, int g, int b, int index) {
                ledBuffer[index].SetRGB(r, g, b);
                ledStrip.SetData(ledBuffer);
            }

            /// @brief Sets the color of the whole LED strip
            /// @param r Red (between 0 and 255)
            /// @param g Green (between 0 and 255)
            /// @param b Blue (between 0 and 255)
            void SetColor(int r, int g, int b) {
                for(int i = 0; i < Constants::kStringLength; i++) {
                    ledBuffer[i].SetRGB(r, g, b);
                }
                ledStrip.SetData(ledBuffer);
            }

            /// @brief Sets the color of the whole LED strip
            /// @param color The color to set for the whole LED strip
            void SetColor(frc::Color color) {
                SetColor(frc::Color8Bit(color));
            }

            /// @brief Sets the color of the whole LED strip
            /// @param color The 8bit representation of the color for the whole LED strip
            void SetColor(frc::Color8Bit color) {
                SetColor(color.red, color.green, color.blue);
            }

            /// @brief When ran periodically, scrolls the whole LED strip length in a rainbow pattern
            void Rainbow() {
                for(int i = 0; i < Constants::kStringLength; i++) {
                    const auto pixelHue = (firstPixelHue + (i * 180 / Constants::kStringLength)) % 180;

                    ledBuffer[i].SetHSV(pixelHue, 255, 128);
                }
                
                // Increase by to make the rainbow "move"
                firstPixelHue += 3;
                
                // Check bounds
                firstPixelHue %= 180;

                ledStrip.SetData(ledBuffer);
            }

        private:
            frc::AddressableLED ledStrip { 0 };
            std::array<frc::AddressableLED::LEDData, Constants::kStringLength> ledBuffer;

            int firstPixelHue = 0;
    };
}