#include <Arduino.h>
#include <arduinoFFT.h>
#include <flag_ship.h>
#include "ADCSampler.h"

#define GAIN_PIN    22
#define AR_PIN      23
#define LED1_PIN  5
#define LED2_PIN  21

#define BUTTON_UP_PIN       18
#define BUTTON_DOWN_PIN     19
#define MOTOR_PHASE1_PIN    27
#define MOTOR_PHASE2_PIN    12

ADCSampler *adcSampler = NULL;
I2SSampler *i2sSampler = NULL;

// i2s config for using the internal ADC
i2s_config_t adcI2SConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_LSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

// how many samples to read at once
const int SAMPLE_SIZE = 128;
arduinoFFT FFT = arduinoFFT();
// send data to a remote address
int16_t filtered[SAMPLE_SIZE];
double vReal[SAMPLE_SIZE];
double vImag[SAMPLE_SIZE];
flag_ship flag_state;
int ledState = LOW;
uint8_t state_det;
uint8_t state_flag = 0, last_state_flag=0;
uint32_t state_time = 0;
const uint32_t state_time_hold = 1500UL;
uint32_t timeMils=0;
uint32_t buttonMillis = 0;

void sendData(int16_t *bytes, size_t count)
{
 for(size_t n = 0; n<count; n++)
 {
   filtered[n]= (int16_t) ((float)bytes [n] - ((float)bytes[n-1]*0.97));
   Serial.println(filtered[n]);
   vReal[n] = filtered[n];//samples[i]; 
   vImag[n] = 0.0; 
 }
  FFT.Windowing(vReal, SAMPLE_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
  FFT.Compute(vReal, vImag, SAMPLE_SIZE, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLE_SIZE); /* Compute magnitudes */
  double f_, v_ ;
  FFT.MajorPeak(vReal, SAMPLE_SIZE, 16000, &f_, &v_);
  Serial.print("frequency : ");Serial.print(f_,5);
  Serial.print("\tpower : ");Serial.println(v_,5);
  // sample measured at
  // frequency : 300.88821	power : 442025.10618
  // range of frequency is 305, with +/- 5
  if( v_ >= 100000.f )
  {
    if( (4477.0 <= f_) && (f_ <= 5477.0 ) )
      state_det = HIGH;
    else
      state_det = LOW;
  }
  else
      state_det = LOW;
  state_flag = flag_state.updateState(state_det);
}

// Task to write samples from ADC to our server


void adcWriterTask(void *param)
{
  I2SSampler *sampler = (I2SSampler *)param;
  int16_t *samples = (int16_t *)malloc(sizeof(uint16_t) * SAMPLE_SIZE);
  if (!samples)
  {
    // Serial.println("Failed to allocate memory for samples");
    return;
  }
  while (true)
  {
    int samples_read = sampler->read(samples, SAMPLE_SIZE);
    sendData(samples, samples_read );
    
    if( last_state_flag != state_flag && (state_flag > 0) )
    {
      state_time = millis();
      last_state_flag = state_flag;
    }
    Serial.print("\tstate : ");Serial.println(state_flag);  
    if( millis() - state_time >= state_time_hold )
    {
      last_state_flag = 0;
    }
    switch(last_state_flag)
    {
      case 1:
        digitalWrite(LED1_PIN, HIGH);
        digitalWrite(LED_BUILTIN, HIGH);
        break;
      case 2:
        digitalWrite(LED2_PIN, HIGH);
        break;
      default :
        digitalWrite(LED2_PIN, LOW);
        digitalWrite(LED1_PIN, LOW);
        digitalWrite(LED_BUILTIN, LOW);
        break;
    };
  }
}
void actuatorTask(void *param)
{
    // setup pin
    // button init state
      pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
      pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
      // MOTOR phase state
      pinMode(MOTOR_PHASE1_PIN, OUTPUT);
      pinMode(MOTOR_PHASE2_PIN, OUTPUT);
    // loop
    while(1)
    {
        digitalWrite(MOTOR_PHASE1_PIN, digitalRead(BUTTON_DOWN_PIN)==HIGH ? LOW:HIGH);
        digitalWrite(MOTOR_PHASE2_PIN, digitalRead(BUTTON_UP_PIN)==HIGH ? LOW:HIGH);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void setup()
{
  Serial.begin(115200);
  
  pinMode(GAIN_PIN,OUTPUT);
  digitalWrite(GAIN_PIN, HIGH);
  pinMode(AR_PIN, OUTPUT);
  digitalWrite(AR_PIN, HIGH);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  flag_state.SetStateAndTime(HIGH);
  // input from analog microphones such as the MAX9814 or MAX4466
  // internal analog to digital converter sampling using i2s
  // create our samplers
  adcSampler = new ADCSampler(ADC_UNIT_1, ADC1_CHANNEL_4, adcI2SConfig);

  // set up the adc sample writer task
  TaskHandle_t adcWriterTaskHandle;
  adcSampler->start();
  xTaskCreatePinnedToCore(adcWriterTask, "ADC Writer Task", 4096, adcSampler, 1, &adcWriterTaskHandle, 1);
  // // start sampling from i2s device
  xTaskCreatePinnedToCore(actuatorTask, "actuator task", 4096, NULL, 1, NULL, 0);
}

void loop()
{
  // nothing to do here - everything is taken care of by tasks
}
