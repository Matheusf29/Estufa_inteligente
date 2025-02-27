/*
PROJETO DE ESTUFA AUTOMATIZADA
NO CULTIVO DE DETERMINADOS TIPOS DE PLANTAS OU NA AGRICULTURA DE PRECISÃO
É NECESSÁRIO O CONTROLE RÍGIDO DE VARÍAVEIS COMO TEMPERATURA, UMIDADE, ENTRE OUTROS.
O SEGUINTE PROJETO TEM COMO OBJETIVO A REALIZAÇÃO DESSE CONTROLE.
COMO SENSORES:
PARA A LEITURA DE TEMPERATURA E UMIDADE - DHT22 - (VAI SER SUBSTITUIDO PELA LEITURA DO  JOYSTICK)
COMO ATUADORES: 
DISPLAY SSD1306
LED RGB
MATRIZ DE LED - INDICAR IRRIGAÇÃO, ventilação e iluminação
*/

//Incluindo as bibliotecas
#include "hardware/pio.h"  
#include "hardware/i2c.h" 
#include "pico/stdlib.h" 
#include "ssd1306.h"   
#include <stdlib.h> 
#include <stdio.h>
#include "font.h"  
#include "hardware/adc.h" 
#include "hardware/pwm.h"
#include "math.h"
#include "ws2818b.pio.h"

// Definições as portas utilizadas 
#define I2C_PORT i2c1    // Porta I2C
#define I2C_SDA 14      
#define I2C_SCL 15     
#define ENDERECO 0x3C   // Endereço do display SSD1306

#define JOYSTICK_X_PIN 26  // GPIO para eixo X
#define JOYSTICK_Y_PIN 27  // GPIO para eixo Y
#define JOYSTICK_PB 22 // Botão do Joystick

#define BOTAO_A 5

#define verde 11            //RGB
#define azul 12              //RGB
#define vermelho 13        //RGB

//Declaração de variáveis utilizadas durante o código

volatile uint32_t last_time = 0; 
bool leitura = true, cor = true;
ssd1306_t ssd;  
uint16_t temp, umid, lux, umid_solo;
//float temp, umidade;    
uint slice_r, slice_b;

//Etapa 1: Leitura dos sensores - Temperatura, umidade
void ler_DHT22(uint16_t *temp, uint16_t *umid);
//Etapa 2: Leitura dos sensores - Luminosidade, umidade do solo
void ler_ldr_FC(uint16_t *lux, uint16_t *umid_solo);
//Etapa 3: Alertas no led
void led_dht22();
void led_ldr_fc();
//Etapa 4: Alertas no display
void display();
//Etapa 5: Animações para indicar o controle das ações
void irrigacao();
void iluminacao();
void ventilar();

/* Função para indexar as posições dos led da matriz */
int getIndex(int x, int y) {
    if (x % 2 == 0) {
        return  24 - (x * 5 + y);
    } else {
        return 24 - (x * 5 + (4 - y));
    }
}

//CONFIGURANDO A MATRIZ DE LEDS com a biblioteca ws2818b
// Definição da estrutura de cor para cada LED da matriz
struct pixel_t {
    uint8_t R, G, B;
};
typedef struct pixel_t npLED_t;
npLED_t leds[25];
// PIO e state machine para controle dos LEDs 
PIO np_pio;
uint sm;

// Função para atualizar os LEDs da matriz e salvar o estado "buffer"
void buffer() {
    for (uint i = 0; i < 25; i++) {
        pio_sm_put_blocking(np_pio, sm, leds[i].R);
        pio_sm_put_blocking(np_pio, sm, leds[i].G);
        pio_sm_put_blocking(np_pio, sm, leds[i].B);
    }
}

// Função de inicializar a matriz de LEDs com a ws2818b.
void iniciar_matriz() {
    uint offset = pio_add_program(pio0, &ws2818b_program);
    np_pio = pio0;
    sm = pio_claim_unused_sm(np_pio, true);
    ws2818b_program_init(np_pio, sm, offset, 7, 800000.f);

    for (uint i = 0; i < 25; i++) {
        leds[i].R = leds[i].G = leds[i].B = 0;
    }
    buffer();
}

// Função para configurar a cor dos LEDs da matriz
void led_cor(const uint indice, const uint8_t r, const uint8_t g, const uint8_t b) {
    leds[indice].R = r;
    leds[indice].G = g;
    leds[indice].B = b;
}

// Função para desligar todos os LEDs
void desliga() {
    for (uint i = 0; i < 25; ++i) { // Percorre todos os LEDs
        led_cor(i, 0, 0, 0); // Define a cor preta (desligado) para cada LED
    }
    buffer(); // Atualiza o estado dos LEDs
}

// Função para leitura do sensor de temperatura e umidade
void ler_DHT22(uint16_t *temp, uint16_t *umid){
    // Leitura do valor do eixo X do joystick - corresponde a temperatura (-40° a 80°C)
    adc_select_input(0); // Seleciona o canal ADC para o eixo X
    sleep_us(20);                     // Pequeno delay para estabilidade
    *temp = adc_read();         // Lê o valor do eixo X (0-4095)
    *temp = (*temp) * 8/1000 + 12; // Lógica para a conversão em tempertura 0 a 33 C para simplificação
    
    // Leitura do valor do eixo Y do joystick - corresponde a umidade (0 a 100%)
    adc_select_input(1); // Seleciona o canal ADC para o eixo Y
    sleep_us(20);                     // Pequeno delay para estabilidade
    *umid = adc_read();         // Lê o valor do eixo Y (0-4095)
    *umid = (*umid) * 100/4094; // De 0 a 100%

}

//Função para leitura do ldr e do sensor de umidade do solo (FC28)
void ler_ldr_FC(uint16_t *lux, uint16_t *umid_solo){
    // Leitura do valor do eixo X do joystick - corresponde a temperatura (-40° a 80°C)
    adc_select_input(0); // Seleciona o canal ADC para o eixo X
    sleep_us(20);                     // Pequeno delay para estabilidade
    *lux = adc_read();         // Lê o valor do eixo X (0-4095)
    // Leitura do valor do eixo Y do joystick - corresponde a umidade (0 a 100%)
    adc_select_input(1); // Seleciona o canal ADC para o eixo Y
    sleep_us(20);                     // Pequeno delay para estabilidade
    *umid_solo = adc_read();         // Lê o valor do eixo Y (0-4095)
    *umid_solo = adc_read();         // Lê o valor do eixo Y (0-4095)
    *umid_solo = (*umid_solo) * 100/4094; // De 0 a 100%
}

//Inicializar os pinos do led RGB
void iniciar_rgb() {
    gpio_init(verde);
    gpio_set_dir(verde, GPIO_OUT);
    gpio_put(verde, 0);

    gpio_init(vermelho);
    gpio_set_dir(vermelho, GPIO_OUT);
    gpio_put(vermelho, 0);

    gpio_init(azul);
    gpio_set_dir(azul, GPIO_OUT);
    gpio_put(azul, 0);
    
    //vermelho e azul como saida pwm
    //configure_pwm(vermelho, &slice_r, level_r);
    //configure_pwm(azul, &slice_b, level_b);
}

// Inicializando e configurando o display SSD1306 
void config_display() {
    // Inicializa a comunicação I2C com velocidade de 400 kHz
    i2c_init(I2C_PORT, 400000); 

    // Configurando os pino para função I2C
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);  
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);

    // Ativando o resistor pull-up
    gpio_pull_up(I2C_SDA);   
    gpio_pull_up(I2C_SCL); 

    ssd1306_init(&ssd, WIDTH, HEIGHT, false, ENDERECO, I2C_PORT); // Inicializa o display SSD1306
    ssd1306_config(&ssd);           // Configura o display
    ssd1306_fill(&ssd, false);      // Limpa
    ssd1306_rect(&ssd, 1, 1, 122, 58, cor, !cor); 
    ssd1306_send_data(&ssd);        // Atualiza o display
}

void botaoA(bool *leitura){
    *leitura = !(*leitura);
}

// Callback chamado quando ocorre interrupção em algum botão
void botao_callback(uint gpio, uint32_t eventos) {
    // Obtém o tempo atual em us
    uint32_t current_time = to_us_since_boot(get_absolute_time());
    // Verifica se passou tempo suficiente desde o último evento
    if (current_time - last_time > 200000) // 200 ms
    {
        last_time = current_time; // Atualiza o tempo do último evento
        if (gpio == BOTAO_A) { //  Botão A foi pressionado
            botaoA(&leitura);
        }
}}

// Função para inicializar os botões
void iniciar() {
    // Inicializa os pinos
    gpio_init(BOTAO_A);
    // Configurando como entrada
    gpio_set_dir(BOTAO_A, GPIO_IN); 
    // Resistor de pull-up
    gpio_pull_up(BOTAO_A);
    // Interrupções para os botões
    gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, botao_callback);
    
    //Inicializa o adc para os eixos x e y do joystck
    adc_init();
    adc_gpio_init(JOYSTICK_X_PIN);
    adc_gpio_init(JOYSTICK_Y_PIN); 
}
  

void display(){
    // Exibição inicial no display OLED
    fflush(stdout); 
    ssd1306_fill(&ssd, !cor);   // Limpa o display
    ssd1306_rect(&ssd, 2, 2, 125, 61, cor, !cor); // Desenha a borda que vai alterar de acordo com a variavel cor
    char texto[16];

    if(leitura){
        snprintf(texto, sizeof(texto), "Temp: %dC",temp);
        ssd1306_draw_string(&ssd, texto, 25, 10);
        snprintf(texto, sizeof(texto), "Um: %d",umid);
        ssd1306_draw_string(&ssd, texto, 25, 35);
    } else{
        snprintf(texto, sizeof(texto), "Lum: %d ",lux);
        ssd1306_draw_string(&ssd, texto, 25, 10);
        snprintf(texto, sizeof(texto), "Solo: %d ",umid_solo);
        ssd1306_draw_string(&ssd, texto, 25, 35);
    }
    
    ssd1306_send_data(&ssd); // Atualizar o display
}

void main() {

    iniciar_matriz();
    stdio_init_all();
    iniciar();
    iniciar_rgb();
    config_display();
    //printf("ola\n");
    //sleep_ms(10); 

// Loop infinito 
while (true) {

    if(leitura){
        ler_DHT22(&temp, &umid);
        //printf("Temperatura = %d, Umidade = %d\n", temp, umid); 
        led_dht22();
        if((umid < 40)){
            irrigacao();
            //printf("irrigação\n");
        }
        if(temp > 30){
            ventilar();
           }

    } else {
        ler_ldr_FC(&lux, &umid_solo);
        //printf("Luminosidade = %d, Umidade do solo = %d\n", lux, umid_solo);
        led_ldr_fc();
        if(lux < 1900){
        iluminacao();   
    }
    if((umid_solo < 40)){
        irrigacao();
        //printf("irrigação\n");
    }
    }
    display();
    desliga();
    sleep_ms(10);
} return;
}


/*Para a simulação, como só temos 3 valores (0, 2047, 4095)
faremos as alterações no código restrito a esses valores 
*/
void led_dht22(){
    if((temp > 30)&&(umid < 40)){
        gpio_put(vermelho, 1);
        gpio_put(verde, 0);
    } else if(((temp > 30)&&(umid > 40)) || ((temp < 30)&&(umid < 40))){
        gpio_put(vermelho, 1);
        gpio_put(verde, 1);
    } else {
        gpio_put(verde, 1); 
        gpio_put(vermelho, 0);}
}

void led_ldr_fc(){
    if((lux < 1900)&&(umid_solo < 40)){
        gpio_put(vermelho, 1);
        gpio_put(verde, 0);
    } else if(((lux > 1900)&&(umid_solo < 40)) || ((lux < 1900)&&(umid_solo > 40))){
        gpio_put(vermelho, 1);
        gpio_put(verde, 1);
    } else {
        gpio_put(verde, 1); 
        gpio_put(vermelho, 0);}
}

void irrigacao(){

    int mat1[5][5][3] = {
        {{0, 0, 200}, {0, 0, 0}, {0, 0, 200}, {0, 0, 0}, {0, 0, 200}},
        {{0, 0, 200}, {0, 0, 0}, {0, 0, 200}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 200}, {0, 0, 0}, {0, 0, 200}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 200}, {0, 0, 0}, {0, 0, 200}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 200}}
    
    };

    // Exibir na matriz
    for (int linha = 0; linha < 5; linha++) {
        for (int cols = 0; cols < 5; cols++) {
            int posicao = getIndex(linha, cols);
            led_cor(posicao, mat1[linha][cols][0], mat1[linha][cols][1], mat1[linha][cols][2]);
        }
    }
    buffer();

}

void iluminacao(){
    int mat1[5][5][3] = {
        {{0, 0, 0}, {0, 0, 0}, {200, 200, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {200, 200, 0}, {200, 200, 0}, {200, 200, 0}, {0, 0, 0}},
        {{200, 200, 0}, {200, 200, 0}, {200, 200, 0}, {200, 200, 0}, {200, 200, 0}},
        {{0, 0, 0}, {200, 200, 0}, {200, 200, 0}, {200, 200, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {200, 200, 0}, {0, 0, 0}, {0, 0, 0}}
    
    };

    // Exibir na matriz
    for (int linha = 0; linha < 5; linha++) {
        for (int cols = 0; cols < 5; cols++) {
            int posicao = getIndex(linha, cols);
            led_cor(posicao, mat1[linha][cols][0], mat1[linha][cols][1], mat1[linha][cols][2]);
        }
    }
    buffer();
}

void ventilar(){
    int mat1[5][5][3] = {
        {{0, 0, 0}, {200, 200, 200}, {0, 0, 0}, {200, 200, 200}, {0, 0, 0}},
        {{200, 200, 200}, {200, 200, 200}, {0, 0, 0}, {200, 200, 200}, {200, 200, 200}},
        {{0, 0, 0}, {0, 0, 0}, {200, 200, 200}, {0, 0, 0}, {0, 0, 0}},
        {{200, 200, 200}, {200, 200, 200}, {0, 0, 0}, {200, 200, 200}, {200, 200, 200}},
        {{0, 0, 0}, {200, 200, 200}, {0, 0, 0}, {200, 200, 200}, {0, 0, 0}}
    
    };

    // Exibir na matriz
    for (int linha = 0; linha < 5; linha++) {
        for (int cols = 0; cols < 5; cols++) {
            int posicao = getIndex(linha, cols);
            led_cor(posicao, mat1[linha][cols][0], mat1[linha][cols][1], mat1[linha][cols][2]);
        }
    }
    buffer();


}