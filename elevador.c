// Sistema de Elevador com Contagem de Passageiros
// 
// descrição: simulação de um elevador com capacidade para 9 passageiros, controlado por botões
// para entrada, saída e reset. exibe a contagem no display OLED, usa matriz de LEDs para
// mostrar números de 0 a 9, LED RGB para indicar status e buzzer para alertas.
//
// desenvolvido por José Vinicius

#include "pico/stdlib.h"            // funções padrão do Pico SDK
#include "hardware/gpio.h"          // controle de pinos GPIO
#include "hardware/i2c.h"           // comunicação I2C para o display OLED SSD1306
#include "hardware/pio.h"           // interface PIO para a matriz de LEDs WS2812
#include "hardware/pwm.h"           // controle PWM para o buzzer
#include "lib/ssd1306.h"            // biblioteca para controle do display OLED SSD1306
#include "generated/ws2812.pio.h"   // programa PIO gerado para comunicação com WS2812
#include "FreeRTOS.h"               // núcleo do FreeRTOS
#include "task.h"                   // gerenciamento de tarefas no FreeRTOS
#include "semphr.h"                 // semáforos e mutexes do FreeRTOS
#include "stdio.h"                  // funções de entrada/saída padrão
#include <string.h>                 // manipulação de strings

// definições de pinos e constantes
#define BOTAO_A 5                   // GPIO para botão A (entrada de passageiro)
#define BOTAO_B 6                   // GPIO para botão B (saída de passageiro)
#define WS2812_PIN 7                // GPIO para matriz de LEDs WS2812
#define MAX_PASSAGEIROS 9           // capacidade máxima do elevador
#define BUZZER 10                   // GPIO para buzzer
#define LED_G 11                    // GPIO do LED RGB verde
#define LED_B 12                    // GPIO do LED RGB azul
#define LED_R 13                    // GPIO do LED RGB vermelho
#define I2C_SDA 14                  // GPIO para pino SDA do I2C 
#define I2C_SCL 15                  // GPIO para pino SCL do I2C 
#define BUTTON_JOYSTICK 22          // GPIO para botão do joystick (reset)
#define I2C_PORT i2c1               // porta I2C usada para o display OLED
#define OLED_ADDRESS 0x3C           // endereço I2C do display OLED SSD1306
#define WIDTH 128                   // largura do display OLED
#define HEIGHT 64                   // altura do display OLED

// variáveis globais
ssd1306_t disp;                     // estrutura para o display OLED
SemaphoreHandle_t xContagemSem;     // semáforo de contagem para gerenciar vagas
SemaphoreHandle_t xResetSem;        // semáforo binário para sinalizar reset
SemaphoreHandle_t xDisplayMutex;    // mutex para proteger acesso ao display
uint8_t passageiros = 0;            // contagem atual de passageiros
uint pwm_slice;                     // slice PWM usado pelo buzzer

// padrões numéricos para a matriz 5x5 (0 a 9)
const uint8_t padroes[10][5][5] = {
    // 0
    {{0,1,1,1,0}, 
     {1,0,0,0,1}, 
     {1,0,0,0,1}, 
     {1,0,0,0,1}, 
     {0,1,1,1,0}},
    // 1
    {{0,0,1,0,0}, 
     {0,1,1,0,0}, 
     {0,0,1,0,0}, 
     {0,0,1,0,0}, 
     {0,1,1,1,0}},
    // 2
    {{0,1,1,1,0}, 
     {0,0,0,1,0}, 
     {0,1,1,1,0}, 
     {0,1,0,0,0}, 
     {0,1,1,1,0}},
    // 3
    {{0,1,1,1,0}, 
     {0,0,0,1,0}, 
     {0,1,1,1,0}, 
     {0,0,0,1,0}, 
     {0,1,1,1,0}},
    // 4
    {{0,1,0,1,0}, 
     {0,1,0,1,0}, 
     {0,1,1,1,0}, 
     {0,0,0,1,0}, 
     {0,0,0,1,0}},
    // 5
    {{0,1,1,1,0}, 
     {0,1,0,0,0}, 
     {0,1,1,1,0}, 
     {0,0,0,1,0}, 
     {0,1,1,1,0}},
    // 6
    {{0,0,1,1,0}, 
     {0,1,0,0,0}, 
     {0,1,1,1,0}, 
     {0,1,0,1,0}, 
     {0,1,1,1,0}},
    // 7
    {{0,1,1,1,1}, 
     {0,0,0,0,1}, 
     {0,0,0,1,0}, 
     {0,0,1,0,0}, 
     {0,1,0,0,0}},
    // 8
    {{0,1,1,1,0}, 
     {0,1,0,1,0}, 
     {0,1,1,1,0}, 
     {0,1,0,1,0}, 
     {0,1,1,1,0}},
    // 9
    {{0,1,1,1,0}, 
     {0,1,0,1,0}, 
     {0,1,1,1,0}, 
     {0,0,0,1,0}, 
     {0,1,1,1,0}}
};

// protótipos de funções
void vTaskEntrada(void *params);                                  // tarefa para gerenciar entrada de passageiros via botão A
void vTaskSaida(void *params);                                    // tarefa para gerenciar saída de passageiros via botão B
void vTaskReset(void *params);                                    // tarefa para resetar o sistema via botão do joystick
void atualizarDisplay(const char* mensagem1, const char* mensagem2, uint8_t passageiros); // atualiza o display OLED com mensagens e contagem
void setLedColor(uint8_t passageiros);                            // configura a cor do LED RGB com base na contagem de passageiros
uint pwm_init_buzzer(uint8_t gpio);                               // inicializa o PWM para o buzzer com frequência específica
void beepCheio(void);                                             // emite um beep curto quando o elevador está cheio
void beepReset(void);                                             // emite dois beeps curtos para indicar reset do sistema
void setMatrizLeds(uint8_t numero);                               // atualiza a matriz de LEDs para exibir número de 0 a 9
void clearMatrizLeds(void);                                       // limpa a matriz de LEDs, desligando todos os pixels
static inline void put_pixel(uint32_t pixel_grb);                 // envia um pixel RGB à matriz WS2812
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b); // converte valores RGB em formato de 32 bits
void gpio_callback(uint gpio, uint32_t events);                   // função de interrupção para o botão do joystick

// função auxiliar para enviar um pixel à matriz 
static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u); // envia o valor RGB ao PIO
}

// função auxiliar para converter valores RGB em formato de 32 bits para a matriz
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b); // combina R, G, B em um único valor
}

// inicializa o PWM para o buzzer 
uint pwm_init_buzzer(uint8_t gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM); // configura o pino para PWM
    uint slice_num = pwm_gpio_to_slice_num(gpio); // obtém o slice PWM associado ao pino
    uint clk_div = clock_get_hz(clk_sys) / (2000 * 4096); // calcula divisor para 2000 Hz
    pwm_set_clkdiv(slice_num, clk_div); // define o divisor de clock
    pwm_set_wrap(slice_num, 4095); // define o valor de wrap (resolução do PWM)
    pwm_set_enabled(slice_num, true); // ativa o PWM
    return slice_num; // retorna o slice 
}

// configura a cor do LED RGB com base no número de passageiros
void setLedColor(uint8_t passageiros) {
    if (passageiros == 0) { // azul (manutenção)
        gpio_put(LED_R, 0); gpio_put(LED_G, 0); gpio_put(LED_B, 1);
    } else if (passageiros <= 7) { // verde (1 a 7 passageiros)
        gpio_put(LED_R, 0); gpio_put(LED_G, 1); gpio_put(LED_B, 0);
    } else if (passageiros == 8) { // amarelo (8 passageiros)
        gpio_put(LED_R, 1); gpio_put(LED_G, 1); gpio_put(LED_B, 0);
    } else { // vermelho (9 passageiros)
        gpio_put(LED_R, 1); gpio_put(LED_G, 0); gpio_put(LED_B, 0);
    }
}

// emite um beep curto quando o elevador está cheio
void beepCheio() {
    pwm_set_gpio_level(BUZZER, 2048); // 50% duty cycle
    vTaskDelay(pdMS_TO_TICKS(200)); // mantém ligado por 200ms
    pwm_set_gpio_level(BUZZER, 0); // desliga o buzzer
}

// emite dois beeps para indicar reset do elevador
void beepReset() {
    beepCheio(); // primeiro beep
    vTaskDelay(pdMS_TO_TICKS(100)); // pausa de 100ms
    beepCheio(); // segundo beep
}

// atualiza a matriz de LEDs para exibir o número de passageiros (0 a 9)
void setMatrizLeds(uint8_t numero) {
    int pixel_map[5][5] = { // mapeamento de índices da matriz WS2812
        {24, 23, 22, 21, 20}, // linha 1: índices dos LEDs
        {15, 16, 17, 18, 19}, // linha 2: índices dos LEDs
        {14, 13, 12, 11, 10}, // linha 3: índices dos LEDs
        {5,  6,  7,  8,  9},  // linha 4: índices dos LEDs
        {4,  3,  2,  1,  0}   // linha 5: índices dos LEDs
    };
    uint32_t pixels[25] = {0}; // array para armazenar os valores RGB dos LEDs
    uint32_t cor; // cor a ser aplicada na matriz

    // define a cor com base no número de passageiros
    if (numero == 0) {
        cor = urgb_u32(0, 0, 8); // azul (manutenção)
    } else if (numero <= 7) {
        cor = urgb_u32(0, 8, 0); // verde (1 a 7)
    } else if (numero == 8) {
        cor = urgb_u32(8, 8, 0); // amarelo (8)
    } else {
        cor = urgb_u32(8, 0, 0); // vermelho (9)
    }

    // aplica o padrão do número na matriz
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            if (padroes[numero][i][j]) {
                pixels[pixel_map[i][j]] = cor; // define a cor para o pixel ativo
            }
        }
    }

    // envia os valores RGB para a matriz de LEDs
    for (int i = 0; i < 25; i++) put_pixel(pixels[i]);
}

// limpa a matriz de LEDs (desliga todos os pixels)
void clearMatrizLeds() {
    for (int i = 0; i < 25; i++) put_pixel(0); // envia valor 0 para todos os pixels
}

// atualiza o display OLED com mensagens e contagem de passageiros
void atualizarDisplay(const char* mensagem1, const char* mensagem2, uint8_t passageiros) {
    if (xSemaphoreTake(xDisplayMutex, portMAX_DELAY) == pdTRUE) { // obtém o mutex
        ssd1306_fill(&disp, 0); // limpa o display
        ssd1306_rect(&disp, 0, 0, WIDTH, HEIGHT, true, false); // desenha borda
        ssd1306_draw_string(&disp, mensagem1, (WIDTH - strlen(mensagem1) * 8) / 2, 16); // mensagem 1 centralizada
        ssd1306_draw_string(&disp, mensagem2, (WIDTH - strlen(mensagem2) * 8) / 2, 32); // mensagem 2 centralizada
        char buffer[16]; // buffer para a contagem
        sprintf(buffer, "Passageiros: %d", passageiros); // formata a contagem
        ssd1306_draw_string(&disp, buffer, (WIDTH - strlen(buffer) * 8) / 2, 48); // exibe contagem
        ssd1306_send_data(&disp); // atualiza o display
        xSemaphoreGive(xDisplayMutex); // libera o mutex
    }
}

// função de interrupção para o botão do joystick com debounce
void gpio_callback(uint gpio, uint32_t events) {
    static uint32_t last_press = 0; // armazena o tempo do último pressionamento
    uint32_t current_time = to_ms_since_boot(get_absolute_time()); // obtém o tempo atual
    if (current_time - last_press < 200) return; // debounce de 200ms
    last_press = current_time; // atualiza o tempo do último pressionamento

    if (gpio == BUTTON_JOYSTICK && events == GPIO_IRQ_EDGE_FALL) { // verifica botão do joystick
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xResetSem, &xHigherPriorityTaskWoken); // sinaliza reset
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken); // cede controle se necessário
    }
}

// tarefa para gerenciar entrada de passageiros
void vTaskEntrada(void *params) {
    while (true) {
        if (gpio_get(BOTAO_A) == 0) { // verifica pressionamento do botão A
            if (xSemaphoreTake(xContagemSem, 0) == pdTRUE) { // tenta ocupar uma vaga
                passageiros++; // incrementa contagem
                atualizarDisplay("Passageiro", "entrou", passageiros); // atualiza display
                setLedColor(passageiros); // atualiza LED RGB
                setMatrizLeds(passageiros); // atualiza matriz de LEDs
            } else { // elevador cheio
                atualizarDisplay("Elevador", "Cheio!", passageiros); // exibe mensagem
                beepCheio(); // emite beep
            }
            vTaskDelay(pdMS_TO_TICKS(200)); // debounce de 200ms
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // pequena pausa para evitar uso excessivo da CPU
    }
}

// tarefa para gerenciar saída de passageiros
void vTaskSaida(void *params) {
    while (true) {
        if (gpio_get(BOTAO_B) == 0) { // verifica pressionamento do botão B
            if (passageiros > 0) { // verifica se há passageiros
                xSemaphoreGive(xContagemSem); // libera uma vaga
                passageiros--; // decrementa contagem
                atualizarDisplay("Passageiro", "saiu", passageiros); // atualiza display
                setLedColor(passageiros); // atualiza LED RGB
                setMatrizLeds(passageiros); // atualiza matriz de LEDs
            }
            vTaskDelay(pdMS_TO_TICKS(200)); // debounce de 200ms
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // pequena pausa para evitar uso excessivo da CPU
    }
}

// tarefa para resetar o elevador
void vTaskReset(void *params) {
    while (true) {
        if (xSemaphoreTake(xResetSem, portMAX_DELAY) == pdTRUE) { // aguarda sinal de reset
            passageiros = 0; // zera contagem
            while (uxSemaphoreGetCount(xContagemSem) < MAX_PASSAGEIROS) { // libera todas as vagas
                xSemaphoreGive(xContagemSem);
            }
            atualizarDisplay("Elevador", "Resetado", passageiros); // atualiza display
            setLedColor(passageiros); // atualiza LED RGB
            clearMatrizLeds(); // limpa matriz de LEDs
            beepReset(); // emite dois beeps
        }
    }
}

// função principal: inicializa os periféricos e cria as tarefas
int main() {
    stdio_init_all(); // inicializa comunicação USB para depuração
    sleep_ms(2000); // aguarda 2s para estabilizar a inicialização

    // inicialização do I2C e display OLED
    i2c_init(I2C_PORT, 400 * 1000); // configura I2C a 400kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // define SDA como função I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // define SCL como função I2C
    gpio_pull_up(I2C_SDA); // ativa pull-up no SDA
    gpio_pull_up(I2C_SCL); // ativa pull-up no SCL
    ssd1306_init(&disp, WIDTH, HEIGHT, false, OLED_ADDRESS, I2C_PORT); // inicializa display
    ssd1306_config(&disp); // configura display
    ssd1306_fill(&disp, 0); // limpa display
    ssd1306_rect(&disp, 0, 0, WIDTH, HEIGHT, true, false); // desenha borda
    ssd1306_draw_string(&disp, "Elevador", (WIDTH - 8 * 8) / 2, 16); // exibe "Elevador"
    ssd1306_draw_string(&disp, "Iniciado", (WIDTH - 8 * 8) / 2, 32); // exibe "Iniciado"
    ssd1306_send_data(&disp); // atualiza display

    // inicialização dos botões
    gpio_init(BOTAO_A); gpio_set_dir(BOTAO_A, GPIO_IN); gpio_pull_up(BOTAO_A); // botão A
    gpio_init(BOTAO_B); gpio_set_dir(BOTAO_B, GPIO_IN); gpio_pull_up(BOTAO_B); // botão B
    gpio_init(BUTTON_JOYSTICK); gpio_set_dir(BUTTON_JOYSTICK, GPIO_IN); gpio_pull_up(BUTTON_JOYSTICK); // joystick

    // inicialização do LED RGB
    gpio_init(LED_R); gpio_set_dir(LED_R, GPIO_OUT); // LED vermelho
    gpio_init(LED_G); gpio_set_dir(LED_G, GPIO_OUT); // LED verde
    gpio_init(LED_B); gpio_set_dir(LED_B, GPIO_OUT); // LED azul
    setLedColor(0); // define cor azul na inicialização

    // inicialização do buzzer com PWM
    pwm_slice = pwm_init_buzzer(BUZZER); // configura PWM no pino do buzzer

    // inicialização da matriz de LEDs WS2812
    PIO pio = pio0; // usa PIO0 para WS2812
    uint offset = pio_add_program(pio, &ws2812_program); // carrega programa WS2812
    ws2812_program_init(pio, 0, offset, WS2812_PIN, 800000, false); // inicializa WS2812 a 800kHz

    // criação dos semáforos e mutex
    xContagemSem = xSemaphoreCreateCounting(MAX_PASSAGEIROS, MAX_PASSAGEIROS); // semáforo para vagas
    xResetSem = xSemaphoreCreateBinary(); // semáforo para reset
    xDisplayMutex = xSemaphoreCreateMutex(); // mutex para display

    // configuração da interrupção do joystick
    gpio_set_irq_enabled_with_callback(BUTTON_JOYSTICK, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // criação das tarefas do FreeRTOS
    xTaskCreate(vTaskEntrada, "Entrada", configMINIMAL_STACK_SIZE + 128, NULL, 1, NULL); // tarefa para entrada
    xTaskCreate(vTaskSaida, "Saida", configMINIMAL_STACK_SIZE + 128, NULL, 1, NULL); // tarefa para saída
    xTaskCreate(vTaskReset, "Reset", configMINIMAL_STACK_SIZE + 128, NULL, 1, NULL); // tarefa para reset

    vTaskStartScheduler(); // inicia o escalonador do FreeRTOS
    while (true); 
}