# Sistemas Embarcados — SPI, I2C e UART

Projeto da disciplina Sistemas Embarcados 2026.2 utilizando a STM32 NUCLEO-L476RG, o módulo ADC/DAC PCF8591 por I2C e uma matriz de LEDs com controlador MAX7219 por SPI. O controle é realizado por comandos enviados pelo terminal serial USART2.

## Hardware

- STM32 NUCLEO-L476RG
- Módulo PCF8591 com LDR, termistor e potenciômetro
- Matriz de LEDs com controlador MAX7219
- Protoboard, jumpers e cabo USB

## Funcionalidades

- Leitura do LDR em AIN0, termistor em AIN1 e potenciômetro em AIN3.
- Controle da saída DAC do PCF8591 com valores de 0 a 255.
- Exibição na matriz de LEDs.
- Comandos de teste e limpeza da matriz.
- Recepção UART e transferências I2C e SPI por interrupções.
- Máquinas de estados para gerenciar as operações de comunicação.

## Conexões

| Pino da NUCLEO-L476RG | Sinal |
|---|---|
| PB8 | PCF8591 SCL |
| PB9 | PCF8591 SDA |
| PA5 | MAX7219 CLK |
| PA7 | MAX7219 DIN |
| PB6 | MAX7219 CS / LOAD |
| PA2 | USART2 TX |
| PA3 | USART2 RX |
| GND | Terra comum dos módulos |

A alimentação deve seguir as especificações dos módulos, respeitando a compatibilidade dos níveis lógicos. O MAX7219 recebe dados sem utilizar MISO; o CS é controlado por software em PB6.

## Terminal serial

Configuração: **115200 baud, 8 bits de dados, sem paridade e 1 bit de parada (8N1)**.

Envie os comandos terminados por Enter, respeitando maiúsculas e minúsculas.

| Comando | Função |
|---|---|
| `Read_AIN0` ou `LDR` | Ler o LDR |
| `Read_AIN1` ou `Temp` | Ler o termistor |
| `Read_AIN3` ou `Volt` | Ler o potenciômetro |
| `Set_DAC_<valor>` | Definir o DAC entre 0 e 255 |
| `LED_TEST` | Testar a matriz de LEDs |
| `LED_CLEAR` | Limpar a matriz |
| `HELP` | Mostrar os comandos disponíveis |

Exemplo:

```text
Set_DAC_128
```

As leituras ADC são códigos de 8 bits. A conversão para temperatura, tensão ou outras grandezas depende da referência e das características dos sensores.

## Periféricos

- **I2C1:** PCF8591 no endereço de 7 bits `0x48`.
- **SPI1:** mestre, transmissão em uma linha, dados de 8 bits e prescaler 64.
- **USART2:** 115200 baud, 8N1.

## Compilação e execução

1. Importe a pasta como projeto existente no STM32CubeIDE.
2. Selecione a configuração Debug e compile.
3. Conecte a NUCLEO-L476RG pelo ST-LINK e grave o firmware.
4. Abra a porta serial virtual do ST-LINK.
5. Envie `HELP` para consultar os comandos e `LED_TEST` para testar a matriz.

O nome interno `PFC8591_I2C_Projeto` foi mantido para preservar a configuração da IDE. Os drivers SPI estão incluídos em `Drivers/STM32L4xx_HAL_Driver`.

As pastas Debug e Release e os binários de compilação são ignorados pelo Git.

## Origem

Adaptado do [projeto I2C original](https://github.com/jeanf009/SE-2026.2---Projeto-4-I2C-), acrescentando a integração SPI com o MAX7219.

Repositório: [projeto-embarcados-spi](https://github.com/notmarsim/projeto-embarcados-spi).

## Alunos

- Ian Teixeira
- Jean Felipe
- Márcio Henrique