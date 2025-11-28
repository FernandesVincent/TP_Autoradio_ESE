# TP_Autoradio_ESE

## Groupe
FERNANDES Vincent
CAILLAUD Paul

## 2. Le GPIO Expander et le VU-Metre

### 2.1 Configuration

1. En regardant sur la page 5 de l'extrait du Kicad on voit que le composant est le MCP23S17-E/SO.  
    [datasheet](https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf)  
    16 bits I/O Expander with serial interface. High speed SPI interface, 10MHz maximum.

2. En regardant les entrees du MCP, on regarde les pins VU_X et on regarde les pins correspondants sur le STM32.  
On peut ensuite aller sur l'IOC du STM32 et on remarque que le seul SPI disonible, par exemple pour VU_MISO est SPI3_MISO. On utilise donc le **SPI3**.  

3. Dans STM32CubeIDE, il faut donc activer le SPI3, verifier les broches MISO, MOSI, SCK et CS. Il faut de plus activer un GPIO pour activer le RESET.


## 3. Le CODEC Audio SGTL5000

### 3.1 Configuration Prealables

1. Sachant que l'on a le module de 32 pins, pour l'I2C, on utilise les pins numeros 27 pour SDA et 29 pour SCL.
Sur le schematic on a que I2C_SDA correspond a la pin PB10 et la pin PB11 correspond a I2C_SDA.  

### 3.2 Configuration du CODEC par l'I2C

1. On observe à l'oscilloscope la clock suivante à la fréquence d'environ 12.3MHz:

![](/drawio/MCLK.jpg)

## 5. Filtre RC 

1. On a le montage suivant: 

![](/drawio/circuit_rc.png)

On calcule l'équation différencielle telle que : 

![](/drawio/equadiff.drawio.png)

2. Équation de récurrence :

![](/drawio/recurrence.png)



