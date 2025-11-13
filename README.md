# TP_Autoradio_ESE

## 2. Le GPIO Expander et le VU-Metre

### 2.1 Configuration

1. En regardant sur la page 5 de l'extrait du Kicad on voit que le composant est le MCP23S17-E/SO.  
    [datasheet](https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf)  
    16 bits I/O Expander with serial interface. High speed SPI interface, 10MHz maximum.

2. En regardant les entrees du MCP, on regarde les pins VU_X et on regarde les pins correspondants sur le STM32.  
On peut ensuite aller sur l'IOC du STM32 et on remarque que le seul SPI disonible, par exemple pour VU_MISO est SPI3_MISO. On utilise donc le **SPI3**.  

3. Dans STM32CubeIDE, il faut donc activer le SPI3, verifier les broches MISO, MOSI, SCK et CS. Il faut de plus activer un GPIO pour activer et desactiver le SPI et un autre GPIO pour le RESET.