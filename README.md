

<div align="center">
    <img src="https://github.com/user-attachments/assets/337c76d6-a1cc-414f-bd90-866fe408b844" alt="V-CC_Color_Basica_TextoAzul" style="width:50%;"/>
</div>

# Projeto Final - Introdução a Sistemas Embarcados

## Autores
- Ivan da Silva Filho
- Ygor de Almeida Pereira

## Descrição do Projeto
Este projeto apresenta um sistema embarcado desenvolvido para monitorar parâmetros essenciais de uma aeronave, incluindo:
- Pressão e temperatura externas;
- Nível de combustível;
- Inclinação e aceleração da aeronave;
- Controle dos flaps.

O sistema utiliza sensores especializados e fornece feedback por meio de um display e de um alarme sonoro. O objetivo principal é garantir a operação segura da aeronave com a exibição de dados em tempo real e alertas em situações críticas.

## Hardware Utilizado

### Dispositivos de Entrada
- **MPU6050-TH** (acelerômetro e giroscópio)
- **BMP280** (sensor de pressão e temperatura)
- **HC-SR04** (sensor ultrassônico)
- **Potenciômetro 10k**
- **Botão**

### Dispositivos de Saída
- **HS96L03W2C03** (display)
- **Buzzer**
- **Servo Motor SG-90**
- **LED** (verde)

### Periféricos
- GPIO
- ADC
- TIMERS
- NVIC
- I2C
- DMA
- PWM

### Diagrama Esquemático
<img width="1161" alt="diagrama" src="https://github.com/user-attachments/assets/a1ea4dba-a85d-4ab9-87db-ac3febb3c2e3" />





## Software

### Visão Geral do Sistema
O sistema realiza as seguintes funções:
1. **Monitoramento de Pressão e Temperatura**:
   - Utiliza o **BMP280** para medir pressão (kPa) e temperatura (°C) externas e calcula a altitude (m) com base nesses valores.
   - Emite alerta sonoro caso a temperatura ultrapasse valores críticos.

2. **Monitoramento de Aceleração e Giro**:
   - Usa o **MPU6050-TH** para medir aceleração (g) nos eixos X, Y e Z, e variação angular (°/s).
   - Calcula e exibe a inclinação da aeronave com base nos dados coletados.

3. **Medição de Nível de Combustível**:
   - Usa o **HC-SR04** para medir o volume de combustível (L) no tanque, baseado na distância percorrida por ondas ultrassônicas.

4. **Controle dos Flaps**:
   - Implementado por um potenciômetro, controlando o movimento de um servo motor que ajusta os flaps.

### Configurações dos Periféricos
- **I2C1**: Fast Mode (400 kHz)
- **ADC1 (IN0)**: Conversão Contínua (12 bits)
- **TIM1**: PWM CH1 para Servo Motor (50Hz)
- **TIM2**: Interrupção em 15.625Hz
- **TIM3**: Medição do tempo das ondas ultrassônicas
- **TIM6**: Interrupção em 0.5Hz
- **GPIO**:
  - **Output**: PA5 (LED), PA6 (Trigger), PB0 (Buzzer)
  - **Input**: PA7 (Echo)
  - **Interrupt**: PC13 (Botão)
- **DMA**: I2C1RX em Modo Normal

### Fluxograma
<img src="https://github.com/user-attachments/assets/026f682d-05e4-4d2c-9bf6-80573952431f" alt="Fluxograma" style="width:70%;"/>

### Interfaces de Display
- Tela de inicialização.
- Feedback do giroscópio.
- Feedback do acelerômetro.
- Feedback de ângulo, altitude e nível de combustível.

## Como Executar
1. Configure o hardware conforme o diagrama esquemático.
2. Faça o upload do firmware para o microcontrolador.
3. Alimente o sistema e observe os dados no display.
4. Realize os ajustes necessários para testar cada funcionalidade.

Sinta-se à vontade para contribuir com melhorias e relatórios de problemas!
![inicializacao](https://github.com/user-attachments/assets/7e48a202-2cc8-4a6a-9daa-15c21c846f7a)

![giroscopio (1)](https://github.com/user-attachments/assets/94ad38d8-515c-40e0-9176-f1fe0b005225)

![acelerometro](https://github.com/user-attachments/assets/0c675352-d013-4506-9d1b-142819104f74)

![angulo](https://github.com/user-attachments/assets/7dd89e6a-546b-42cb-8e9f-cce3baee7e9c)


