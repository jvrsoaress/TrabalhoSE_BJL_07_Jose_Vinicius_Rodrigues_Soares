<img width=100% src="https://capsule-render.vercel.app/api?type=waving&color=02A6F4&height=120&section=header"/>
<h1 align="center">Embarcatech - Projeto Integrado - BitDogLab </h1>

## Objetivo do Projeto

Um sistema de elevador utilizando o Raspberry Pi Pico W na plaquinha BitDogLab, simulando o controle de acesso de até 9 passageiros. O sistema gerencia entradas e saídas via botões, exibe a contagem em tempo real no display OLED, sinaliza o status com matriz de LEDs e LED RGB, e emite alertas sonoros com buzzer, promovendo segurança e acessibilidade em espaços físicos.

## 🗒️ Lista de requisitos

- **Leitura de botões (A, B e Joystick):** Botão A (entrada de passageiro, GPIO 5), Botão B (saída de passageiro, GPIO 6), Botão do Joystick (reset, GPIO 22);
- **Utilização da matriz de LEDs:** Exibe números de 0 a 9 correspondentes à contagem de passageiros, com cores azul (0), verde (1-7), amarelo (8), vermelho (9);
- **Utilização de LED RGB:** Sinaliza estados do elevador: azul (0 passageiros), verde (1-7), amarelo (8), vermelho (9);
- **Display OLED (SSD1306):** Exibe mensagens (exp: "Passageiro entrou" ou "Elevador Cheio!") e contagem de passageiros;
- **Utilização do buzzer:** Emite beep curto quando o elevador está cheio e dois beeps curtos ao resetar pelo Botão do Joystick;
- **Estruturação do projeto:** Código em C no VS Code, usando Pico SDK e FreeRTOS, com comentários detalhados;
- **Técnicas implementadas:** I2C, PIO, PWM, interrupções, FreeRTOS com semáforos e mutex, debounce via software.
  

## 🛠 Tecnologias

1. **Microcontrolador:** Raspberry Pi Pico W (na BitDogLab).
2. **Display OLED SSD1306:** 128x64 pixels, conectado via I2C (GPIO 14 - SDA, GPIO 15 - SCL).
3. **Botão do Joystick:** GPIO 22 (Reset).
4. **Botão A:** GPIO 5 (Simula a entrada de um passageiro no elevador).
5. **Botão B:** GPIO 6 (Simula a saída de um passageiro no elevador).
6. **Matriz de LEDs:** WS2812 (GPIO 7).
7. **LED RGB:** GPIOs 11 (verde), 12 (azul), 13 (vermelho).
8. **Buzzer:** GPIO 10.
10. **Linguagem de Programação:** C.
11. **Frameworks:** Pico SDK, FreeRTOS.


## 🔧 Funcionalidades Implementadas:

**Funções dos Componentes**

- **Matriz de LEDs (WS2812):** Exibe números de 0 a 9 em uma matriz 5x5, com brilho ajustado (valor 8), usando cores sincronizadas com o LED RGB.
- **LED RGB:** Sinaliza o estado do elevador: azul (0 passageiros), verde (1-7), amarelo (8), vermelho (9).  
- **Display OLED:** Exibe em tempo real:
  - Mensagens como "Passageiro entrou", "Passageiro saiu", "Elevador Cheio!", "Elevador Resetado".
  - Contagem de passageiros (exp: "Passageiros: 8").
- **Buzzer:** Emite beep curto (200ms) quando o elevador está cheio e dois beeps curtos ao resetar.
- **Botões:** 
  - Joystick: Reseta o sistema via interrupção, zerando a contagem e sinalizando.
  - Botão A: Registra entrada de passageiro, com debounce de 200ms.
  - Botão B: Registra saída de passageiro com debounce de 200ms.
- **Técnicas:**
  - Uso de semáforos FreeRTOS: Semáforo de contagem para gerenciar 9 vagas, semáforo binário para reset, mutex para proteger o display.
  - Interrupção: Configurada para o botão do joystick (GPIO 22) com debounce de 200ms.
  - I2C: Comunicação com OLED a 400 kHz.
  - PIO: Controle da matriz WS2812.
  - PWM: Buzzer com frequência de 2000 Hz.

## 🚀 Passos para Compilação e Upload do projeto Ohmímetro com Matriz de LEDs

1. **Instale o Pico SDK:** Configure o ambiente com Pico SDK e FreeRTOS.
2. **Crie uma pasta `build`**:
   ```bash
   mkdir build
   cd build
   cmake ..
   make

3. **Transferir o firmware para a placa:**

- Conectar a placa BitDogLab ao computador via USB.
- Copiar o arquivo .uf2 gerado para o drive da placa.

4. **Testar o projeto**

- Após o upload, desconecte e reconecte a placa.
- Use os botões A e B para simular entrada e saída de passageiros.
- Pressione o botão do joystick para resetar o sistema.

🛠🔧🛠🔧🛠🔧


## 🎥 Demonstração: 

- Para ver o funcionamento do projeto, acesse o vídeo de demonstração gravado por José Vinicius em: 

## 💻 Desenvolvedor
 
<table>
  <tr>
    <td align="center"><img style="" src="https://avatars.githubusercontent.com/u/191687774?v=4" width="100px;" alt=""/><br /><sub><b> José Vinicius </b></sub></a><br />👨‍💻</a></td>
  </tr>
</table>
