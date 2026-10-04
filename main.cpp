#include <Arduino.h>
#include "driver/i2s.h"
#include <arduinoFFT.h>

// Electromagnets
#define MAGNET_LOW 25
#define MAGNET_MID 26
#define MAGNET_HIGH 27

// Mic
#define SCK 18
#define WS 19
#define SD 34
#define I2S_PORT I2S_NUM_0

// Number of mic samples to collect before processing
#define AUDIO_BLOCK_SIZE 512

// Microphone sampling rate used for FFT calculations
#define SAMPLE_RATE 16000


// Electromagnet PWM settings
#define MAGNET_PWM_FREQUENCY 5000
#define MAGNET_PWM_RESOLUTION 8
#define MAGNET_MAX_OUTPUT 242

// Electromagnet PWM channels
#define MAGNET_LOW_CHANNEL 0
#define MAGNET_MID_CHANNEL 1
#define MAGNET_HIGH_CHANNEL 2


// Store real number values used by FFT
float fftReal[AUDIO_BLOCK_SIZE];

// Store imaginary number values used by FFT
float fftImag[AUDIO_BLOCK_SIZE];

// FFT object using sample arrays, sample count, and sample rate
ArduinoFFT<float> FFT = ArduinoFFT<float>(fftReal, fftImag, AUDIO_BLOCK_SIZE, SAMPLE_RATE);



//mic setup prototype
void setupMicrophone();
// FFT processing prototype
void processFFT(int32_t audioSamples[]);
// Frequency level normalization prototype
int normalizeLevel(float level, float minLevel, float maxLevel);
// Electromagnet control prototype
void controlMagnets(int lowOutput, int midOutput, int highOutput);


void setup() {

  Serial.begin(115200);
  delay(500);

  // Set electromagnet pins as outputs
  pinMode(MAGNET_LOW, OUTPUT);
  pinMode(MAGNET_MID, OUTPUT);
  pinMode(MAGNET_HIGH, OUTPUT);

  // Configure PWM frequency and resolution for each electromagnet channel
  ledcSetup(MAGNET_LOW_CHANNEL, MAGNET_PWM_FREQUENCY, MAGNET_PWM_RESOLUTION);
  ledcSetup(MAGNET_MID_CHANNEL, MAGNET_PWM_FREQUENCY, MAGNET_PWM_RESOLUTION);
  ledcSetup(MAGNET_HIGH_CHANNEL, MAGNET_PWM_FREQUENCY, MAGNET_PWM_RESOLUTION);

  // Attach each electromagnet GPIO pin to its PWM channel
  ledcAttachPin(MAGNET_LOW, MAGNET_LOW_CHANNEL);
  ledcAttachPin(MAGNET_MID, MAGNET_MID_CHANNEL);
  ledcAttachPin(MAGNET_HIGH, MAGNET_HIGH_CHANNEL);
 
  

  // Keep electromagnets off for starting
  ledcWrite(MAGNET_LOW_CHANNEL, 0);
  ledcWrite(MAGNET_MID_CHANNEL, 0);
  ledcWrite(MAGNET_HIGH_CHANNEL, 0);

  // Initialize mic and its I2S interface
  setupMicrophone();
  Serial.println("INMP441 initialized.");
}

void loop() {

  // Store one batch of mic samples
  int32_t audioSamples[AUDIO_BLOCK_SIZE];

  // Track how many mic samples have been collected
  int samplesCollected = 0;

  // Keep reading until one full batch is collected
  while (samplesCollected < AUDIO_BLOCK_SIZE) {

    // Create buffer to temp hold audio samples
    int32_t samples[64];

    // Stores how many bytes of mic data were received
    size_t bytes_read = 0;

    // Read incoming mic data from corresponding pins, grab sample count,
    // and wait until data is available
    i2s_read(I2S_PORT, samples, sizeof(samples), &bytes_read, portMAX_DELAY);

    // Convert bytes received into number of 32-bit audio samples
    int sample_count = bytes_read / sizeof(int32_t);

    for (int i = 0; i < sample_count; i += 2) {

      // Store the microphone channel and skip the unused channel
      if (samplesCollected < AUDIO_BLOCK_SIZE) {

        audioSamples[samplesCollected] = samples[i];
        samplesCollected++;
      }
    }
  }

  // Process completed audio batch for frequency content
  processFFT(audioSamples);

 
}


// Process collected mic samples into low, mid, and high frequency levels
void processFFT(int32_t audioSamples[]) {

  // Add all mic samples together to calculate their average
  int64_t sampleSum = 0;

  // Loop through every sample in the current audio batch
  for (int i = 0; i < AUDIO_BLOCK_SIZE; i++) {

    // Add current sample to running total
    sampleSum += audioSamples[i];
  }

  // Find average value of all samples to get center point of current audio signal
  int32_t sampleAverage = sampleSum / AUDIO_BLOCK_SIZE;


  // Prepare every audio sample for FFT processing
  for (int i = 0; i < AUDIO_BLOCK_SIZE; i++) {

    // Remove average center from current sample and reduce 32-bit mic value to its useful 24-bit range
    fftReal[i] = (float)(((int64_t)audioSamples[i] - sampleAverage) / 256);
    
    // Mic samples have no imaginary value before FFT so set each to zero
    fftImag[i] = 0;
  }


  // Apply Hamming window to reduce frequency leakage between FFT frequencies
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);

  // Convert mic samples from time domain into frequency domain
  FFT.compute(FFTDirection::Forward);

  // Convert FFT results into magnitude of each frequency
  FFT.complexToMagnitude();

  // Store total squared magnitude found inside each frequency band
  float lowEnergySum = 0;
  float midEnergySum = 0;
  float highEnergySum = 0;

  // Track how many FFT frequency bins are included in each band
  int lowBinCount = 0;
  int midBinCount = 0;
  int highBinCount = 0;

  // Only first half of FFT results contains usable frequencies from 0 Hz to 8000 Hz
  for (int i = 1; i < AUDIO_BLOCK_SIZE / 2; i++) {

    // Calculate frequency represented by current FFT bin
    float frequency = ((float)i * SAMPLE_RATE) / AUDIO_BLOCK_SIZE;

    // Add frequencies from 50 Hz up to 250 Hz into low band
    if (frequency >= 50 && frequency < 250) {

      // Square current magnitude and add it to total low frequency energy
      lowEnergySum += fftReal[i] * fftReal[i];

      // Count current frequency bin as part of low band
      lowBinCount++;
    }

    // Add frequencies from 250 Hz up to 2000 Hz into mid band
    else if (frequency >= 250 && frequency < 2000) {

      // Square current magnitude and add it to total mid frequency energy
      midEnergySum += fftReal[i] * fftReal[i];

      // Count current frequency bin as part of mid band
      midBinCount++;
    }

    // Add frequencies from 2000 Hz up to 8000 Hz into high band
    else if (frequency >= 2000 && frequency <= 8000) {

      // Square current magnitude and add it to total high frequency energy
      highEnergySum += fftReal[i] * fftReal[i];

      // Count current frequency bin as part of high band
      highBinCount++;
    }

  }
    // Store final overall activity level for each frequency band
  float lowLevel = 0;
  float midLevel = 0;
  float highLevel = 0;

  // Calculate average low frequency energy and convert it back to magnitude
  if (lowBinCount > 0) {

    lowLevel = sqrt(lowEnergySum / lowBinCount);
  }

  // Calculate average mid frequency energy and convert it back to magnitude
  if (midBinCount > 0) {

    midLevel = sqrt(midEnergySum / midBinCount);
  }

  // Calculate average high frequency energy and convert it back to magnitude
  if (highBinCount > 0) {

    highLevel = sqrt(highEnergySum / highBinCount);
  }

  // Convert each raw frequency level into 0 to 255 control value
  int lowOutput = normalizeLevel(lowLevel, 1500000, 13000000);
  int midOutput = normalizeLevel(midLevel, 300000, 12000000);
  int highOutput = normalizeLevel(highLevel, 100000, 5500000);
        
  // Send normalized frequency levels to electromagnets
  controlMagnets(lowOutput, midOutput, highOutput);
  

  // Print frequency band values slowly instead of every audio batch
  static unsigned long lastPrintTime = 0;

  if (millis() - lastPrintTime >= 1000) {

    Serial.print("LOW: ");
    Serial.print(lowLevel, 0);
    Serial.print(" -> ");
    Serial.print(lowOutput);

    Serial.print("    MID: ");
    Serial.print(midLevel, 0);
    Serial.print(" -> ");
    Serial.print(midOutput);

    Serial.print("    HIGH: ");
    Serial.print(highLevel, 0);
    Serial.print(" -> ");
    Serial.println(highOutput);

    lastPrintTime = millis();
  }
}


// Convert raw frequency level into 0 to 255 control value
int normalizeLevel(float level, float minLevel, float maxLevel) {

  // Keep values below minimum level at zero
  if (level <= minLevel) {

    return 0;
  }

  // Keep values above maximum level at full strength
  if (level >= maxLevel) {

    return 255;
  }

  // Calculate where current level falls between minimum and maximum
  float normalizedLevel = (level - minLevel) / (maxLevel - minLevel);

  // Convert normalized 0 to 1 value into 0 to 255 range
  int outputLevel = normalizedLevel * 255;

  return outputLevel;
}


// Control electromagnet strength using normalized frequency levels
void controlMagnets(int lowOutput, int midOutput, int highOutput) {

  // Limit low magnet to maximum allowed PWM strength
  if (lowOutput > MAGNET_MAX_OUTPUT) {

    lowOutput = MAGNET_MAX_OUTPUT;
  }

  // Limit mid magnet to maximum allowed PWM strength
  if (midOutput > MAGNET_MAX_OUTPUT) {

    midOutput = MAGNET_MAX_OUTPUT;
  }

  // Limit high magnet to maximum allowed PWM strength
  if (highOutput > MAGNET_MAX_OUTPUT) {

    highOutput = MAGNET_MAX_OUTPUT;
  }

  // Send low frequency strength to low electromagnet
  ledcWrite(MAGNET_LOW_CHANNEL, lowOutput);

  // Send mid frequency strength to mid electromagnet
  ledcWrite(MAGNET_MID_CHANNEL, midOutput);

  // Send high frequency strength to high electromagnet
  ledcWrite(MAGNET_HIGH_CHANNEL, highOutput);
}


// Config/initialize the I2S communication for the mic
void setupMicrophone() {

    // I2S struct. init all fields to 0
    i2s_config_t i2s_config = {};

    // Set ESP32 master and to receive signal
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);

    // Microphone sampling rate (sample/sec)
    i2s_config.sample_rate = 16000;

    // Receive each sample as 32-bit value
    i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;

    // Receive both I2S channels
    i2s_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;

    // Config I2S for communication format
    i2s_config.communication_format = I2S_COMM_FORMAT_I2S;

    // Interrupt to respond to new mic data ready

    i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;

    // DMA buffers for continuous data receiving without relying and CPU and unnecessary delay
    // 8 for good balance of storing our samples without loosing too much and not too much latency
    i2s_config.dma_buf_count = 8;

    // Each buffer to hold 64 audio samples (of 8)
    i2s_config.dma_buf_len = 64;

    // Using esp32 clk source instead, set apll false
    i2s_config.use_apll = false;

    // No audio transmission from master
    i2s_config.tx_desc_auto_clear = false;

    // No fixed master clock
    i2s_config.fixed_mclk = 0;


    // I2S pin config struct
    i2s_pin_config_t pin_config = {};

    // clock pin
    pin_config.bck_io_num = SCK;

    // Word Select pin
    pin_config.ws_io_num = WS;

    // No data is being sent to mic
    pin_config.data_out_num = I2S_PIN_NO_CHANGE;

    // Data in from pin SD
    pin_config.data_in_num = SD;


    // Install the I2S driver using config settings that were just set
    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

    // Set I2S ports to corresponding GPIO
    i2s_set_pin(I2S_PORT, &pin_config);

    // Clear leftover data from buffer
    i2s_zero_dma_buffer(I2S_PORT);
}


