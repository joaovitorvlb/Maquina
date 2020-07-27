#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <AccelStepper.h>
#include <Adafruit_PWMServoDriver.h>
#include <event_groups.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);

#define mainONE_SHOT_TIMER_PERIOD		  40
#define mainTWO_SHOT_TIMER_PERIOD		  100
#define mainTree_SHOT_TIMER_PERIOD		  500
#define mainFour_SHOT_TIMER_PERIOD		  20
#define mainFive_SHOT_TIMER_PERIOD		  357

TimerHandle_t xAutoReloadTimer, xOneShotTimer, xTwoShotTimer, xTreeShotTimer, xFourShotTimer, xFiveShotTimer;
BaseType_t xTimer1Started, xTimer2Started;

#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define DEFAULT_PULSE_WIDTH   1500
#define FREQUENCY             50

#define velocidade_motor1     400
#define aceleracao_motor1     400

#define vel_agulha            300
#define acel_agulha           300

#define vel_retorno_carro     700
#define acel_retorno_carro    700

#define vel_avanco_carro      350
#define acel_avanco_carro     350

#define io_vsv                29         // PA7

#define io_fc_VNT1_1          28         // PA6
#define io_fc_VNT1_2          27         // PA5     (Motor do batedor)
#define io_fc_VNT1_3          26         // PA4     (Injeta a caneta)
#define io_fc_VNT1_4          25         // PA3     (Sopro)

#define io_fc1_1              64       // PK2 digital number
#define io_fc1_2              65       // PK3 digital number

#define io_fc2_1              66       // PK4 digital number
#define io_fc2_2              67       // PK5 digital number

#define io_mag_en             42       // PL7
#define io_mag_step           43       // PL6
#define io_mag_dir            44       // PL5

#define io_mol_en             41       // PG0
#define io_mol_step           40       // PG1
#define io_mol_dir            37       // PC0

#define io_car_en             36       // PC1
#define io_car_step           35       // PC2
#define io_car_dir            19       // PD2

#define io_l293_en1           12       // PB6  pwm do "motor 1" OC1B
#define io_l293_in1           48       // PL1
#define io_ls93_in2           46       // PL3

#define io_l293_en2           11       // PB5  pwm do "motor 1" OC1A
#define io_l293_in3           45       // PL4
#define io_l293_in4           47       // PL2

#define eixo_1                0
#define eixo_2                1
#define dedo                  2
#define lanca                 3
#define cacamba               4

#define BIT_agulha    	     ( 1UL << 0UL )
#define BIT_carro_avan       ( 1UL << 1UL )
#define BIT_carro_ret  	     ( 1UL << 2UL )
#define BIT_lanca        	 ( 1UL << 3UL )
#define BIT_dedo        	 ( 1UL << 4UL )
#define BIT_x     	         ( 1UL << 5UL )
#define BIT_xx               ( 1UL << 6UL )
#define BIT_xxx          	 ( 1UL << 7UL )

EventGroupHandle_t xEventGroup;

typedef struct struct_robo
{
    uint8_t eixo1;
    uint8_t eixo2;
    uint8_t eixo3;
};

typedef struct struct_carro
{
    uint8_t carro1;
    uint8_t carro2;
};

typedef struct xExampleStructure2
{
    uint8_t tempo1;
    uint8_t tempo2;
    uint8_t tempo3;
} Example2_t;


QueueHandle_t MsgRobo;           // Declara mensagem para o robô do tipo QueueHandle_t
QueueHandle_t MsgLanca;          // Declara mensagem para o robô do tipo QueueHandle_t
QueueHandle_t MsgTempoServo;     // Declara mensagem para o tempo servo do tipo QueueHandle_t
QueueHandle_t MsgCarro;

TaskHandle_t TaskHandle_1;
TaskHandle_t TaskHandle_2;
TaskHandle_t TaskHandle_3;

// Definicao pinos STEP e DIR da agulha
AccelStepper MotorAgulha(1, io_mag_step, io_mag_dir);

// Definicao pinos STEP e DIR do carro
AccelStepper MotorCar(1, io_car_step, io_car_dir);

void ProcessoEntrada();
void ProcessoSaida();
void ControlaRobo();
int pulseWidth(int angle);
static void prvOneShotTimerCallback();
static void prvTwoShotTimerCallback();
static void prvTreeShotTimerCallback();
static void prvFourShotTimerCallback();
static void prvFiveShotTimerCallback();

void PwmT1_DutyA(unsigned char duty);
void PwmT1_DutyB(unsigned char duty);
void PwmT1_DutyC(unsigned char duty);
void PwmT1_init();

static unsigned char estado_robo      = 0;
static bool flgg_carro                = 0;
static bool flgg_agulha               = 0;
static bool  flag_ded                 = 1;
static bool flag_tt                   = 0;

void setup()
{
    xEventGroup = xEventGroupCreate();

    xEventGroupSetBits( xEventGroup, BIT_carro_ret);
    xEventGroupClearBits( xEventGroup, BIT_dedo);

    MsgRobo = xQueueCreate( 1, sizeof( struct_robo ));            // Cria a menasgem para o robô
    MsgLanca = xQueueCreate( 1, sizeof( struct_robo ));           // Cria a mensagem para a lança
    MsgTempoServo = xQueueCreate( 1, sizeof( Example2_t ));       // Cria a mensagem para a tempo servo
    MsgCarro = xQueueCreate(1, sizeof(struct_carro));

    // Realiza configirações do PCA9685
    pwm.begin();
    pwm.setPWMFreq(FREQUENCY);

    // Configuracoes iniciais motor de passo da agulha
    MotorAgulha.setMaxSpeed(vel_agulha);
    MotorAgulha.setSpeed(vel_agulha);
    MotorAgulha.setAcceleration(acel_agulha);

    // Configuracoes iniciais motor de passo do carro da agulha
    MotorCar.setMaxSpeed(velocidade_motor1);
    MotorCar.setSpeed(velocidade_motor1);
    MotorCar.setAcceleration(aceleracao_motor1);

    pinMode(io_vsv, OUTPUT);
    digitalWrite(io_vsv, LOW);

    pinMode(io_mag_en, OUTPUT);
    digitalWrite(io_mag_en, HIGH);

    pinMode(io_car_en, OUTPUT);
    digitalWrite(io_car_en, HIGH);

    pinMode(io_mol_en, OUTPUT);
    digitalWrite(io_mol_en, HIGH);

    pinMode(io_l293_in3, OUTPUT);
    pinMode(io_l293_in4, OUTPUT);

    digitalWrite(io_l293_in3, LOW);
    digitalWrite(io_l293_in4, LOW);

    pinMode(38, OUTPUT);
    digitalWrite(38, HIGH);

    pinMode(io_fc2_1, INPUT);
    digitalWrite(io_fc2_1, HIGH);

    pinMode(io_fc2_2, INPUT);
    digitalWrite(io_fc2_2, HIGH);

    pinMode(io_fc_VNT1_1, INPUT);
    digitalWrite(io_fc_VNT1_1, HIGH);

    pinMode(io_fc_VNT1_2, OUTPUT);          // Motor do batedor
    digitalWrite(io_fc_VNT1_2, HIGH);

    pinMode(io_fc_VNT1_3, OUTPUT);          // Injeta a caneta
    digitalWrite(io_fc_VNT1_3, HIGH);

    pinMode(io_fc_VNT1_4, OUTPUT);          // Sopro
    digitalWrite(io_fc_VNT1_4, HIGH);

    pinMode(io_fc1_1, OUTPUT);             // Saída para solenoide que joga o cigarro
    digitalWrite(io_fc1_1, HIGH);

    pinMode(io_fc1_2, INPUT);
    digitalWrite(io_fc1_2, LOW);

    PwmT1_init();
    PwmT1_DutyA(255);
    PwmT1_DutyB(20);
    PwmT1_DutyC(20);

    Serial.begin(9600);

    // inicializa a comunicação serial a 9600 bits/s:
    Serial.begin(9600);

    xOneShotTimer = xTimerCreate( "OneShot",	   			  	// Text name for the software timer - not used by FreeRTOS.
  								  mainONE_SHOT_TIMER_PERIOD,	// The software timer's period in ticks.
  								  pdFALSE,						// Setting uxAutoRealod to pdFALSE creates a one-shot software timer.
  								  0,						    // This example does not use the timer id.
  								  prvOneShotTimerCallback );	// The callback function to be used by the software timer being created.

    xTwoShotTimer = xTimerCreate( "TwoShot",	   			   	// Text name for the software timer - not used by FreeRTOS.
  								  mainTWO_SHOT_TIMER_PERIOD,	// The software timer's period in ticks.
  								  pdFALSE,						// Setting uxAutoRealod to pdFALSE creates a one-shot software timer.
  								  1,						    // This example does not use the timer id.
  								  prvTwoShotTimerCallback );	// The callback function to be used by the software timer being created.

    xTreeShotTimer = xTimerCreate( "TreeShot",	   	  		    // Text name for the software timer - not used by FreeRTOS.
  								  mainTree_SHOT_TIMER_PERIOD,	// The software timer's period in ticks.
  								  pdFALSE,						// Setting uxAutoRealod to pdFALSE creates a one-shot software timer.
  								  2,						    // This example does not use the timer id.
  								  prvTreeShotTimerCallback );	// The callback function to be used by the software timer being created.

    xFourShotTimer = xTimerCreate( "FourShot",	   		  	    // Text name for the software timer - not used by FreeRTOS.
  								  mainFour_SHOT_TIMER_PERIOD,	// The software timer's period in ticks.
  								  pdFALSE,						// Setting uxAutoRealod to pdFALSE creates a one-shot software timer.
  								  3,						    // This example does not use the timer id.
  								  prvFourShotTimerCallback );	// The callback function to be used by the software timer being created.

    xFiveShotTimer = xTimerCreate( "FiveShot",	   		  	    // Text name for the software timer - not used by FreeRTOS.
  								  mainFive_SHOT_TIMER_PERIOD,	// The software timer's period in ticks.
  								  pdFALSE,						// Setting uxAutoRealod to pdFALSE creates a one-shot software timer.
  								  4,						    // This example does not use the timer id.
  								  prvFiveShotTimerCallback );	// The callback function to be used by the software timer being created.

    // Cria task para as tarefas do processo de entrada
    xTaskCreate(
                  ProcessoEntrada,                           // Define cowback para de chamada da task
                  (const portCHAR *) "ProcessoEntrada",      // Define um nome para possível indicação
                  1000,                                      // Stack size de 128B de memória heap
                  NULL,                                      // Não passa nenhum parametro
                  2,                                         // Prioridade de nível 2
                  &TaskHandle_1 );

    // Cria task para as tarefas do processo de entrada
    xTaskCreate(
                  ControlaRobo,                            // Define cowback para de chamada da task
                  (const portCHAR *) "ControlaRobo",       // Define um nome para possível indicação
                  1000,                                    // Stack size de 128B de memória heap
                  NULL,                                    // Não passa nenhum parametro
                  2,                                       // Prioridade de nível 2
                  &TaskHandle_2 );

    // Cria task para as tarefas do processo de entrada
    xTaskCreate(
                  ProcessoSaida,                           // Define cowback para de chamada da task
                  (const portCHAR *) "ProcessoSaida",      // Define um nome para possível indicação
                  1000,                                    // Stack size de 128B de memória heap
                  NULL,                                    // Não passa nenhum parametro
                  2,                                       // Prioridade de nível 2
                  &TaskHandle_3 );
}

int pulseWidth(int angle)       // Função do tipo int
{
    int pulse_wide, analog_value;
    pulse_wide   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
    return analog_value;
}

void loop()
{
  // Vazio. Todo trabalho já foi feito nas Tasks.
}

/*---------------------- Tasks ---------------------*/
void ProcessoEntrada()  // Isto é uma tarefa
{

    TickType_t xDelay                   = 1000 / portTICK_PERIOD_MS;
    TickType_t xDelay_1ms               = 30 / portTICK_PERIOD_MS;

    static uint8_t ex1_pegando          = 48;
    static uint8_t ex1_15p              = 22;
    static uint8_t ex1_soltando         = 10;

    static uint8_t ex2_levantado        = 57;
    static uint8_t ex2_pegando          = 15;
    static uint8_t ex2_soltando         = 0;

    // BaseType_t xStatus;
    struct_robo Robo;
    struct_carro Carro;
    Example2_t TmpServo;
    static EventBits_t  result;

    pinMode(io_fc2_1, INPUT);
    digitalWrite(io_fc2_1, HIGH);

    Robo.eixo1 =  ex1_15p;    // Eixo 1 na posição do processo
    Robo.eixo2 = ex2_levantado;   // Eixo 2 na posição baixado
    Robo.eixo3 = 0;
    xQueueOverwrite( MsgRobo, &Robo );            // Dispara posição inicial para a task dos eixos
    pwm.setPWM( cacamba, 0, pulseWidth(160));     // Despeja a caçamba

    for (;;)
    {
        result = xEventGroupGetBits( xEventGroup );      // Verifica a atualicação dos flags da mquina

        switch(estado_robo)
        {
            case 0:          // inicia subida do robo
            {
                Robo.eixo1 = ex1_15p;                           // Robo no fim da palha
                Robo.eixo2 = ex2_levantado;             // Robo levantado
                Robo.eixo3 = 0;

                estado_robo = 1;
                break;
            }

            case 1:          // Coloca o robo fora do caminho da caneta
            {
                Robo.eixo1 = ex1_pegando;               // Robo no fim da palha
                Robo.eixo2 = ex2_levantado;              // Robo levantado
                Robo.eixo3 = 0;

                estado_robo = 2;
                break;
            }


            case 2:         // Desce a ventosa e pega a palha
            {
                Robo.eixo1 = ex1_pegando;               // Robo no inicio da palha
                Robo.eixo2 = ex2_pegando;                // Robo baixado (pegando a palha)
                Robo.eixo3 = 0;

                TmpServo.tempo1 = 40;                      // informa o tempo lento para o servo

                estado_robo = 3;
                break;
            }

            case 3:          // Levanta o robo com a palha na ventosa
            {
                Robo.eixo1 = ex1_pegando;            // Robo no inicio da palha
                Robo.eixo2 = ex2_levantado;           // Robo levantado
                Robo.eixo3 = 0;

                estado_robo = 4;
                break;
            }

            case 4:                                      // Aguarda o carro caso esteja ocupado
            {
                flag_tt = 1;                             // Inicia a varredura de 1 milisegundos
                if((result & BIT_carro_ret) != 0 )       // Verifica se o carro terminou o ciclo anterior
                {
                    estado_robo = 5;
                }
                break;
            }

            case 5:          // Robo inicia giro para o levar a palha
            {
                flag_tt = 0;            // inicia a varredura de 2 segundos

                Robo.eixo1 = ex1_soltando;                          // Robo no fim da palha
                Robo.eixo2 = ex2_levantado;            // Robo levantado
                Robo.eixo3 = 0;

                TmpServo.tempo1 = 30;                            // Tempo para servo mais rápido
                xQueueOverwrite( MsgTempoServo, &TmpServo );     // Envia para task a atualização de tempo

                estado_robo = 6;
                break;
            }

            case 6:                     // inicia descida do robo e solta a palha
            {
                Robo.eixo1 = ex1_soltando;         // Robo no fim da palha
                Robo.eixo2 = ex2_soltando;         // Robo baixado (soltando a palha)
                Robo.eixo3 = 0;

                xQueueOverwrite( MsgTempoServo, &TmpServo );
                estado_robo = 7;
                break;
            }

            case 7:
            {
                Carro.carro1 = 2;
                xQueueOverwrite( MsgCarro, &Carro );              // Dispara o avanco do carro
                estado_robo = 8;
                break;
            }

            case 8:
            {
                flag_tt = 1;
                if((result & BIT_carro_avan) != 0 )                       // Aguarda o carro até a agulha encaixar na palha
                {
                    xEventGroupClearBits( xEventGroup, BIT_carro_avan);
                    estado_robo = 9;
                }
                break;
            }

            case 9:          // inicia subida do robo
            {
                pwm.setPWM( cacamba, 0, pulseWidth(00));    // Joga o fumo na palha
                xTimerStart( xTwoShotTimer, 1 );            // joga o fumo na caçamba
                Robo.eixo1 = ex1_soltando;                  // Robo no fim da palha
                Robo.eixo2 = ex2_levantado;                 // Robo levantado
                Robo.eixo3 = 0;

                flag_tt = 0;
                estado_robo = 10;
                break;
            }

            case 10:                    // Coloca o robo fora do caminho da caneta
            {
                xEventGroupSetBits( xEventGroup, BIT_lanca);   // Faz a caneta avançar
                Robo.eixo1 = ex1_15p;                          // posiciona o robo fora do caminho da caneta
                Robo.eixo2 = ex2_levantado;                    // Robo levantado
                Robo.eixo3 = 0;

                estado_robo = 11;
                break;
            }

            case 11:                    // Aguarda o carro caso esteja ocupado
            {
                flag_tt = 1;            // Inicia a varredura de 1 milisegundos
                xTimerStart( xFiveShotTimer, 4 );    // joga o fumo na caçamba
                estado_robo = 12;
                break;
            }

            case 12:                    // Braço avançado 15 graus e aguardando overflow do timer five
            {
               break;
            }

            case 13:                    // Robo inicia giro apra o fim do processo
            {
                Robo.eixo1 = ex1_pegando;            // Robo no fim da palha
                Robo.eixo2 = ex2_levantado;           // Robo levantado
                Robo.eixo3 = 0;

                TmpServo.tempo1 = 30;                          // informa o tempo rápido para o servo
                xQueueOverwrite( MsgTempoServo, &TmpServo );
                estado_robo = 2;
                break;
            }
        }

        if (flag_tt == 0)
        {
            xQueueOverwrite( MsgRobo, &Robo );
            vTaskDelay(xDelay);
        }
        vTaskDelay(xDelay_1ms);        
    }
}

void ControlaRobo()
{
    static int   estado_agulha       = 0;
    static int   estado_carro        = 0;
    static bool  flag_sens           = 0;
    static int   count_a             = 0;
    static bool  flag_can            = 0;

    static bool falg_timer = 0;

    struct_carro Carro;
    BaseType_t StatusCarro;

    static EventBits_t  result;

    uint16_t analogValue_a;
    uint16_t analogValue_b;

    // xEventGroupSetBits( xEventGroup, mainFIRST_TASK_BIT );
    // xEventGroupClearBits( xEventGroup, mainSECOND_TASK_BIT );

    for(;;)
    {
        result = xEventGroupGetBits( xEventGroup );

        StatusCarro = xQueueReceive( MsgCarro, &Carro, NULL);

        if (StatusCarro == pdPASS )
        {
           estado_carro = 2;
        }

        if((result & BIT_agulha) != 0 )
        {
            estado_agulha = 1;
            xEventGroupClearBits( xEventGroup, BIT_agulha);
        }

        switch (estado_agulha)
        {
            case 0:
                digitalWrite( io_mag_en, HIGH);
                flgg_agulha = 0;
            break;

            case 1:
                digitalWrite( io_mag_en, LOW);
                MotorAgulha.moveTo(41000);

                if ((digitalRead( io_fc1_2 ) == 1) && (flag_sens == 0))
                {
                    count_a++;
                    flag_sens = 1;
                }

                if ((digitalRead( io_fc1_2 ) == 0) && (flag_sens == 1))
                {
                    flag_sens = 0;
                }

                if (count_a == 1)
                {
                    digitalWrite(io_fc_VNT1_3, LOW);                     // Faz a caneta injetar plástico na palha
                    if (flag_ded == 1)
                    {
                        flag_ded = 0;
                        xEventGroupSetBits( xEventGroup, BIT_dedo);      // Aciona o dedo para apertar a palha
                        xTimerStart( xTreeShotTimer, 2 );                // Dispara timer para voltar o dedo
                    }
                }

                if (count_a == 2)
                {
                    flag_can = 1;
                }

                if (count_a == 9)
                {
                    digitalWrite(io_fc_VNT1_3, HIGH);                        // Faz a caneta parar de injetar
                }

                if (count_a == 12)
                {
                    xEventGroupClearBits( xEventGroup, BIT_lanca);           // Retorna a caneta
                }

                // if ((MotorAgulha.currentPosition() == 41000) || (count_a == 12))
                if (count_a == 12)
                {
                    MotorAgulha.setCurrentPosition(0);        // Zera a contagem e passo do motor da agulha
                    estado_carro = 5;                         // Manda retornar o carro da agulha
                    estado_agulha = 0;                        // Manda esligar o motor da agulha
                    count_a = 0;                              // Zera a contagem de voltas da agulha
                }
            break;
        }

        switch (estado_carro)
        {
            case 0:                                     // parado com motor livre
                digitalWrite(io_car_en, HIGH);
            break;

            case 1:                                    // parado com motor freado
                digitalWrite(io_car_en, LOW);
            break;

            case 2:                                    // Avança com o carro da agulha
                xEventGroupClearBits( xEventGroup, BIT_carro_ret );   // dispara flag de carro ocupado  
                digitalWrite(io_car_en, LOW);                         // Habilita o drive
                MotorCar.moveTo(-20000);                    // movimentaa até 20000 pulsos anti horário
                
                // Verifica o fim de avanço do carro
                if (digitalRead(io_fc2_2) == 0 || MotorCar.currentPosition() == 20000)       
                {
                    MotorCar.setCurrentPosition(0);
                    xEventGroupSetBits( xEventGroup, BIT_carro_avan );
                    xEventGroupSetBits( xEventGroup, BIT_agulha );
                    MotorCar.setMaxSpeed( vel_retorno_carro );          // Configura retorno veloz 
                    MotorCar.setSpeed( vel_retorno_carro );             // Configura retorno veloz
                    MotorCar.setAcceleration( acel_retorno_carro );     // Configura retorno veloz
                    estado_carro = 3;
                }
            break;

            case 3:                                      // Aguarda o motor a agulha terminar de enrrolar a palha
                digitalWrite(io_car_en, HIGH);           // Permanece com o motor solto
            break;

            case 5:                                     // Retorna o carro da agulha
                digitalWrite(io_car_en, LOW);
                MotorCar.moveTo(20000);

                if (digitalRead(io_fc2_1) == 0 || MotorCar.currentPosition() == 20000)
                {
                    MotorCar.setCurrentPosition(0);
                    estado_carro = 0;
                    flgg_carro = 0;
                    xEventGroupSetBits( xEventGroup, BIT_carro_ret );      // Indica que o carro esta retornando
                    MotorCar.setMaxSpeed( vel_avanco_carro );
                    MotorCar.setSpeed( vel_avanco_carro );
                    MotorCar.setAcceleration( acel_avanco_carro );

                    analogValue_a = analogRead(4);
                    analogValue_b = (analogValue_a / 8) + 1; 

                    xTimerChangePeriod( xOneShotTimer, analogValue_b , 0 );
                    
                    xTimerStart( xOneShotTimer, 0 );               // joga o fumo na caçamba
                    xTimerStart( xFourShotTimer, 3 );              // Desliga o ar do cigarro
                    digitalWrite( io_l293_in3, HIGH);
                    digitalWrite(io_fc_VNT1_2, LOW);              // liga o motor do batedor
                }
            break;
        }

        MotorAgulha.run();
        MotorCar.run();
    }
}

void ProcessoSaida()
{
    static unsigned char pulso1  = 22;
    static unsigned char pulso2  = 0;
    static unsigned char vl1     = 22;
    static unsigned char vl2     = 35;
    static unsigned char tempo   = 0;

    static uint8_t dedo_set      = 0;
    static uint8_t dedo_set_max  = 40;
    static uint8_t dedo_set_min  = 0;
    static uint8_t dedo_value    = 0;

    static uint8_t lanca_set     = 10;

    static uint8_t lanca_v_max   = 90;
    static uint8_t lanca_v_min   = 10;

    static uint8_t lanca_value   = lanca_v_max;

    TickType_t xDelay = 30 / portTICK_PERIOD_MS;
    TickType_t xDelay1 = 40 / portTICK_PERIOD_MS;

    static bool flag_brk = 0;

    BaseType_t StatusRobo;
    struct_robo Robo;
    BaseType_t StatusTmp;
    Example2_t TmpServo;

    pwm.setPWM( eixo_1, 0, pulseWidth(vl1));
    pwm.setPWM( eixo_2, 0, pulseWidth(vl2));
    pwm.setPWM( lanca, 0, pulseWidth(lanca_value));     // Braço da caneta, 0 encostado na plalha e maior é repouso
    pwm.setPWM( dedo, 0, pulseWidth(dedo_set_min));

    // pwm.setPWM( lanca, 0, pulseWidth(0));

    digitalWrite(io_vsv, HIGH);

    static EventBits_t  result;

    for (;;)
    {
        StatusRobo = xQueueReceive( MsgRobo, &Robo,  NULL);

        StatusTmp = xQueueReceive( MsgTempoServo, &TmpServo,  NULL);

        result = xEventGroupGetBits( xEventGroup );

        if (digitalRead(io_fc_VNT1_1) == 1)
        {
            digitalWrite(io_l293_in3, LOW);
            digitalWrite(io_l293_in4, LOW);
            digitalWrite(io_mol_en, HIGH);
            digitalWrite(io_mag_en, HIGH);
            digitalWrite(io_car_en, HIGH);

            digitalWrite(io_fc_VNT1_3, HIGH);
            // digitalWrite(io_fc_VNT1_4, HIGH);       // Desliga o sopro por break

            vTaskSuspend(TaskHandle_1);
            vTaskSuspend(TaskHandle_2);
            
            pulso2 = 58;
            lanca_set = 40;
            flag_brk = 1;
        }
        else
        {
            if( StatusRobo == pdPASS )
            {
                pulso1 = Robo.eixo1;
                pulso2 = Robo.eixo2;
            }

            if (( result & BIT_lanca ) != 0 )
            {
                lanca_set = lanca_v_min;
            }
            else
            {
                lanca_set = lanca_v_max;
            }
        }

        if (flag_brk == 1)
        {
            if( vl2 > 55)
            {
                pulso1 = 0;
            }
        }

        if (( result & BIT_dedo ) != 0 )
        {
            dedo_set = dedo_set_max;
        }
        else
        {
            dedo_set = dedo_set_min;
        }

        if( StatusTmp == pdPASS )
        {
            tempo = TmpServo.tempo1;
            xDelay = tempo / portTICK_PERIOD_MS;
        }

        if( vl1 > pulso1 )
        {
            vl1--;
            pwm.setPWM(eixo_1, 0, pulseWidth(vl1));
            Serial.println(vl1);
        }

        if( vl1 < (pulso1 - 1))
        {
            vl1++;
            pwm.setPWM(eixo_1, 0, pulseWidth(vl1));
            Serial.println(vl1);
        }

        if( vl2 > (pulso2 + 1))              // Avança para soltar a palhe
        {
            if (vl2 > 1)
            {
                vl2--;
                pwm.setPWM(eixo_2, 0, pulseWidth(vl2));
            }
        }

        if(vl2 < pulso2 )               // Levanta o braço
        {
            vl2++;
            pwm.setPWM(eixo_2, 0, pulseWidth(vl2));
        }

        if( dedo_value > dedo_set )    // Abaixa o braço
        {
            dedo_value--;
            pwm.setPWM(dedo, 0, pulseWidth(dedo_value));
        }

        if( dedo_value < dedo_set )
        {
            dedo_value++;
            pwm.setPWM(dedo, 0, pulseWidth(dedo_value));
        }

        if( lanca_value > (lanca_set + 1))
        {
            if (lanca_value > 1)
            {
                lanca_value--;
                pwm.setPWM(lanca, 0, pulseWidth(lanca_value));
            }
        }

        if( lanca_value < (lanca_set - 1))
        {
            lanca_value++;
            pwm.setPWM(lanca, 0, pulseWidth(lanca_value));
        }

        vTaskDelay(xDelay);

        uint16_t anvalue = analogRead(3);
        uint16_t anvalue1;         

        anvalue1 = int(anvalue / 4);

        PwmT1_DutyA(anvalue1);
    }
}

void PwmT1_init()
{
    // PWM modo fast de 8 bits nao invertido
    TCCR1A |= (1 << COM1A1) | (0 << COM1A0) | (1 << WGM10);

    TCCR1B |= (0 << CS11) | (1 << CS10);

    OCR1AL = 0x00;
    OCR1BL = 0x00;
    OCR1CL = 0x00;
    OCR1AH = 0x00;
    OCR1BH = 0x00;
    OCR1CH = 0x00;

    DDRB  |= (1 << DDB5);
}

void PwmT1_DutyA(unsigned char duty)
{
    OCR1AL = duty;
}

void PwmT1_DutyB(unsigned char duty)
{
    OCR1BL = duty;
}

void PwmT1_DutyC(unsigned char duty)
{
    OCR1CL = duty;
}

static void prvOneShotTimerCallback()
{
    digitalWrite(io_l293_in3, LOW);              // desliga o motor do batedor 
}

static void prvTwoShotTimerCallback()
{
    pwm.setPWM(cacamba, 0, pulseWidth(160));           // Volta a caçamba
}

static void prvTreeShotTimerCallback()
{
    xEventGroupClearBits( xEventGroup, BIT_dedo);      // Volta o dedo para soltar a palha enrrolda
    flag_ded = 1;
}

static void prvFourShotTimerCallback()
{
    digitalWrite(io_fc_VNT1_2, HIGH);              // Desliga o sopro por tempo
}

static void prvFiveShotTimerCallback()
{
    estado_robo = 13;
    flag_tt = 0;         // volta o tempo da varredura de 2 segundos
}
