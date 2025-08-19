# mcus
STM32 - ESP32 - RaspberryPi

```mermaid
flowchart LR
  LCD ---|I2C| STM32
  DHT22 ---|GPIO| STM32
  STM32 ---|SPI| ESP32
  ESP32 -. BLE .- RPI
  STM32 ---|CAN| RPI
```
