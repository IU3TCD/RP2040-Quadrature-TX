// ============================================================
// RP2040 - Modulatore SSB/AM/CW phasing (Hilbert) per FST3235 QSE
// Arduino IDE + core Earle Philhower (RP2040)
// AUTOSUFFICIENTE: Vcore 1.20V + clock 250 MHz da codice
//
// core0: DSP (ISR audio, Hilbert, I/Q)   |   core1: I2C slave (comando)
//
// ------------------------------------------------------------
//  PIN USATI  -  4 uscite in quadratura, pilotaggio diretto FST3235
// ------------------------------------------------------------
//   GPIO6  : 0°   = I   (PWM, slice con GPIO7)
//   GPIO7  : 180° = -I  (PWM, stesso slice di GPIO6)
//   GPIO8  : 90°  = Q   (PWM, slice con GPIO9)
//   GPIO9  : 270° = -Q  (PWM, stesso slice di GPIO8)
//   GPIO12 : tasto CW  (input, pull-up; a massa = key down)
//   GPIO16 : pin di TEST (input, pull-up; a massa = forza tono, rilascio = normale)
//   GPIO2  : I2C1 SDA  (slave) | GPIO3 : I2C1 SCL (slave)
//   GPIO28 : ingresso microfono (ADC2)
//   (fasi commutazione 0°/90° FST3235 dal Si5351 ESTERNO)
//
// ------------------------------------------------------------
//  FILTRO RC (doppio stadio, in serie a OGNI uscita PWM, x4 rami)
// ------------------------------------------------------------
//   GPIO --[R1 330R]--+--[R2 220R]--+--> ingresso FST3235
//                     |             |
//                  [C1 47nF]     [C2 220nF]
//                     |             |
//                    GND           GND
//   1° stadio 330R/47nF (fc~10.3kHz), 2° stadio 220R/220nF (fc~3.29kHz)
//   Ordine: fc alta (330R/47nF) PRIMA. Banda voce ~2.4dB@2.7kHz,
//   portante PWM 244kHz attenuata ~60dB, Zout ~550R.
//   I 4 rami (I/-I/Q/-Q) devono essere identici (R,C all'1% o appaiati)
//   Catena tutta accoppiata in DC (necessario per la portante AM = offset DC)
//
// ------------------------------------------------------------
//  I2C  (slave, bus I2C1, indirizzo 0x42)   [reg][val]
// ------------------------------------------------------------
//   REG_MODE     0x01 : 0=USB 1=LSB 2=AM 3=CW
//   REG_SOURCE   0x02 : 0=mic 1=1 tono 2=2 toni
//   REG_CARRIER  0x03 : livello portante AM (1 byte 0..255)
//   REG_MODIDX   0x04 : indice modulazione AM (1 byte 0..255)
//   REG_STATUS   0x10 : lettura -> [mode][source effettiva]
//
//  Modi: USB=I,Q | LSB=q=-q | CW=tono 700Hz keyato (GPIO12, rampa soft)
//        AM=portante(offset DC)+audio su I, Q=0 (portante generata nel DSP)
//  Uscita fissa 20.4 MHz, filtro a quarzo 7 kHz (term. 800R) a valle
//  PWM DAC 10 bit (~244 kHz), fs = 48828.125 Hz
// ============================================================

#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "pico/stdlib.h"
#include <Wire.h>
#include <math.h>

// ---------- Parametri ----------
#define SYS_KHZ     250000
#define VCORE       VREG_VOLTAGE_1_20
#define DECIM       5
#define FS_HZ       48828.125f
#define NTAPS       101
#define DELAY       (NTAPS/2)
#define PWM_WRAP    1023
#define PWM_MID     512

// 4 uscite quadratura: I/-I su slice_i, Q/-Q su slice_q
#define PIN_I       6      // 0°
#define PIN_IN      7      // 180° (-I)  stesso slice di PIN_I
#define PIN_Q       8      // 90°
#define PIN_QN      9      // 270° (-Q)  stesso slice di PIN_Q
#define PIN_MIC     28
#define ADC_CH      2

// ---------- CW key ----------
#define PIN_CW_KEY  12
#define CW_RAMP     25

// ---------- Pin di test (attivo basso) ----------
#define PIN_TEST    16     // a massa = forza tono; rilascio = torna a cmd_source

// ---------- I2C slave (core1) ----------
#define I2C_ADDR    0x42
#define I2C_SDA     2
#define I2C_SCL     3
#define REG_MODE    0x01
#define REG_SOURCE  0x02
#define REG_CARRIER 0x03
#define REG_MODIDX  0x04
#define REG_STATUS  0x10

// ---------- Sorgenti ----------
#define SRC_MIC   0
#define SRC_ONE   1
#define SRC_TWO   2

// ---------- Modi TX ----------
#define MODE_USB  0
#define MODE_LSB  1
#define MODE_AM   2
#define MODE_CW   3

// ---------- Banda ----------
#define BP_LOW      300.0f
#define BP_HIGH     2700.0f
#define IQ_TRIM     1.000f
#define Q           15
#define OUT_SHIFT   2

// ---------- Toni ----------
#define TONE_HZ     1000.0f
#define TONE1_HZ    700.0f
#define TONE2_HZ    1900.0f
#define CW_TONE_HZ  700.0f
#define TONE_AMP    1500

// ---------- Stato ----------
static int32_t hb_q[NTAPS];
static int32_t win[NTAPS];   static int widx = 0;
static int32_t dline[NTAPS]; static int didx = 0;
static int32_t iq_trim_q;

static volatile int source     = SRC_MIC;   // sorgente EFFETTIVA (applicata)
static volatile int cmd_source = SRC_MIC;   // sorgente comandata via I2C (da ripristinare)
static volatile int tx_mode    = MODE_USB;

// AM (regolabili via I2C)
static volatile int32_t carrier_dc  = 300;
static volatile int32_t mod_index_q = (int32_t)(0.8f*(1<<Q));

static volatile int cw_key = 0;
static int32_t cw_env = 0;
static uint32_t cw_inc = 0;

typedef struct { int32_t b0,b1,b2,a1,a2; int32_t z1,z2; } biquad_t;
static biquad_t bp;

static int16_t  sine256[256];
static uint32_t dds1_acc=0, dds1_inc=0, dds2_acc=0, dds2_inc=0;

static uint slice_i, slice_q;
static int  decim_cnt = 0;

// ---------- Sorgente: incremento DDS ----------
static void set_source(int s) {
  source = s;
  float f = (s == SRC_TWO) ? TONE1_HZ : TONE_HZ;
  dds1_inc = (uint32_t)((f / FS_HZ) * 4294967296.0f);
}

// ---------- Hilbert passabanda ----------
static void build_hilbert() {
  float fl = BP_LOW / FS_HZ, fh = BP_HIGH / FS_HZ;
  for (int n = 0; n < NTAPS; n++) {
    int m = n - DELAY;
    float h;
    if (m == 0) h = 0.0f;
    else {
      float wl = 2.0f*(float)M_PI*fl, wh = 2.0f*(float)M_PI*fh;
      h = (cosf(wl*m) - cosf(wh*m)) / ((float)M_PI * m);
    }
    float bm = 0.42f - 0.5f*cosf(2.0f*(float)M_PI*n/(NTAPS-1))
                     + 0.08f*cosf(4.0f*(float)M_PI*n/(NTAPS-1));
    hb_q[n] = (int32_t)lroundf(h * bm * (1 << Q));
  }
  iq_trim_q = (int32_t)lroundf(IQ_TRIM * (1 << Q));
}

// ---------- Bandpass RBJ ----------
static void build_bandpass() {
  float f0 = sqrtf(BP_LOW * BP_HIGH);
  float bw = BP_HIGH - BP_LOW;
  float Qf = f0 / bw;
  float w0 = 2.0f*(float)M_PI*f0/FS_HZ;
  float alpha = sinf(w0)/(2.0f*Qf);
  float cw = cosf(w0);
  float a0 = 1.0f + alpha;
  float b0 = alpha/a0, b2 = -alpha/a0;
  float a1 = (-2.0f*cw)/a0, a2 = (1.0f - alpha)/a0;
  bp.b0=(int32_t)lroundf(b0*(1<<Q)); bp.b1=0;
  bp.b2=(int32_t)lroundf(b2*(1<<Q)); bp.a1=(int32_t)lroundf(a1*(1<<Q));
  bp.a2=(int32_t)lroundf(a2*(1<<Q)); bp.z1=bp.z2=0;
}
static inline int32_t bandpass(int32_t x) {
  int32_t y = (bp.b0*x + bp.z1) >> Q;
  bp.z1 = bp.b1*x - bp.a1*y + bp.z2;
  bp.z2 = bp.b2*x - bp.a2*y;
  return y;
}

static void build_dds() {
  for (int i=0;i<256;i++)
    sine256[i]=(int16_t)(TONE_AMP*sinf(2.0f*(float)M_PI*i/256.0f));
  dds2_inc = (uint32_t)((TONE2_HZ  / FS_HZ) * 4294967296.0f);
  cw_inc   = (uint32_t)((CW_TONE_HZ / FS_HZ) * 4294967296.0f);
  set_source(source);
}

// ---------- Sorgente audio ----------
static inline int32_t read_source() {
  if (tx_mode == MODE_CW) {
    dds1_acc += cw_inc;
    return (int32_t)sine256[dds1_acc>>24];
  }
  switch (source) {
    case SRC_ONE:
      dds1_acc += dds1_inc;
      return (int32_t)sine256[dds1_acc>>24];
    case SRC_TWO:
      dds1_acc += dds1_inc; dds2_acc += dds2_inc;
      return ((int32_t)sine256[dds1_acc>>24] + (int32_t)sine256[dds2_acc>>24]) >> 1;
    case SRC_MIC:
    default:
      return (int32_t)adc_read() - 2048;
  }
}

// ---------- ISR (core0) ----------
static void __not_in_flash_func(sample_isr)() {
  pwm_clear_irq(slice_i);
  if (++decim_cnt < DECIM) return;
  decim_cnt = 0;

  int   mode = tx_mode;
  int32_t x  = read_source();

  x = bandpass(x);

  // ramo Q: Hilbert
  win[widx] = x;
  int base = widx + NTAPS;
  int64_t acc = 0;
  for (int k = 0; k < NTAPS; k++) {
    int idx = base - k; if (idx >= NTAPS) idx -= NTAPS;
    acc += (int64_t)hb_q[k] * win[idx];
  }
  int32_t q = (int32_t)(acc >> Q);
  if (++widx >= NTAPS) widx = 0;

  q = (int32_t)(((int64_t)q * iq_trim_q) >> Q);

  // ramo I: ritardo di DELAY
  dline[didx] = x;
  int rd = didx - DELAY; if (rd < 0) rd += NTAPS;
  int32_t iout = dline[rd];
  if (++didx >= NTAPS) didx = 0;

  // modo TX
  switch (mode) {
    case MODE_USB:
      break;
    case MODE_LSB:
      q = -q;
      break;
    case MODE_CW: {
      int32_t target = cw_key ? (1<<Q) : 0;
      if (cw_env < target) { cw_env += CW_RAMP; if (cw_env > target) cw_env = target; }
      else if (cw_env > target) { cw_env -= CW_RAMP; if (cw_env < target) cw_env = target; }
      iout = (int32_t)(((int64_t)iout * cw_env) >> Q);
      q    = (int32_t)(((int64_t)q    * cw_env) >> Q);
      break;
    }
    case MODE_AM:
      iout = carrier_dc + (int32_t)(((int64_t)iout * mod_index_q) >> Q);
      q = 0;
      break;
  }

  // --- 4 uscite in quadratura 10 bit ---
  int vi = (iout >> OUT_SHIFT) + PWM_MID;   // 0°
  int vq = (q    >> OUT_SHIFT) + PWM_MID;   // 90°
  if (vi<0) vi=0; else if (vi>PWM_WRAP) vi=PWM_WRAP;
  if (vq<0) vq=0; else if (vq>PWM_WRAP) vq=PWM_WRAP;
  int vin = 2*PWM_MID - vi;                 // 180° (-I)
  int vqn = 2*PWM_MID - vq;                 // 270° (-Q)

  pwm_set_gpio_level(PIN_I,  vi);
  pwm_set_gpio_level(PIN_IN, vin);
  pwm_set_gpio_level(PIN_Q,  vq);
  pwm_set_gpio_level(PIN_QN, vqn);
}

// ============================================================
//  CORE 0 - DSP
// ============================================================
void setup() {
  vreg_set_voltage(VCORE);
  delay(10);
  set_sys_clock_khz(SYS_KHZ, true);

  build_bandpass();
  build_hilbert();
  build_dds();
  for (int i=0;i<NTAPS;i++){ win[i]=0; dline[i]=0; }

  adc_init(); adc_gpio_init(PIN_MIC); adc_select_input(ADC_CH);

  gpio_init(PIN_CW_KEY); gpio_set_dir(PIN_CW_KEY, GPIO_IN); gpio_pull_up(PIN_CW_KEY);
  gpio_init(PIN_TEST);   gpio_set_dir(PIN_TEST, GPIO_IN);   gpio_pull_up(PIN_TEST);

  // 4 uscite PWM: I/-I su slice_i, Q/-Q su slice_q
  gpio_set_function(PIN_I,  GPIO_FUNC_PWM);
  gpio_set_function(PIN_IN, GPIO_FUNC_PWM);
  gpio_set_function(PIN_Q,  GPIO_FUNC_PWM);
  gpio_set_function(PIN_QN, GPIO_FUNC_PWM);
  slice_i = pwm_gpio_to_slice_num(PIN_I);   // GPIO6/7
  slice_q = pwm_gpio_to_slice_num(PIN_Q);   // GPIO8/9

  pwm_config cfg = pwm_get_default_config();
  pwm_config_set_wrap(&cfg, PWM_WRAP);
  pwm_config_set_clkdiv(&cfg, 1.0f);
  pwm_init(slice_i, &cfg, false);
  pwm_init(slice_q, &cfg, false);
  pwm_set_gpio_level(PIN_I,  PWM_MID);
  pwm_set_gpio_level(PIN_IN, PWM_MID);
  pwm_set_gpio_level(PIN_Q,  PWM_MID);
  pwm_set_gpio_level(PIN_QN, PWM_MID);
  pwm_set_mask_enabled((1u << slice_i) | (1u << slice_q));

  pwm_clear_irq(slice_i);
  pwm_set_irq_enabled(slice_i, true);
  irq_set_exclusive_handler(PWM_IRQ_WRAP, sample_isr);
  irq_set_enabled(PWM_IRQ_WRAP, true);

  set_source(SRC_MIC);
  cmd_source = SRC_MIC;
}

void loop() {
  cw_key = (gpio_get(PIN_CW_KEY) == 0) ? 1 : 0;

  // pin di test attivo basso: forza tono singolo; al rilascio torna a cmd_source
  static int test_prev = 1;                  // 1 = rilasciato
  int test_now = (gpio_get(PIN_TEST) == 0);  // 1 = premuto (a massa)
  if (test_now && !test_prev) {
    set_source(SRC_ONE);                      // fronte discesa: forza tono
  } else if (!test_now && test_prev) {
    set_source(cmd_source);                   // fronte salita: ripristina I2C
  }
  test_prev = test_now;
}

// ============================================================
//  CORE 1 - I2C slave (comando)
// ============================================================
static uint8_t i2c_buf[4];

void onI2CReceive(int len) {
  int n = 0;
  while (Wire1.available() && n < (int)sizeof(i2c_buf))
    i2c_buf[n++] = Wire1.read();
  while (Wire1.available()) Wire1.read();
  if (n < 2) return;

  switch (i2c_buf[0]) {
    case REG_MODE:
      if (i2c_buf[1] <= MODE_CW) tx_mode = i2c_buf[1];
      break;
    case REG_SOURCE:
      if (i2c_buf[1] <= SRC_TWO) {
        cmd_source = i2c_buf[1];
        // applica subito solo se il pin di test NON è attivo
        if (gpio_get(PIN_TEST) != 0) set_source(i2c_buf[1]);
      }
      break;
    case REG_CARRIER:
      carrier_dc = i2c_buf[1] * 4;
      break;
    case REG_MODIDX:
      mod_index_q = (int32_t)(((int64_t)i2c_buf[1] * (1<<Q)) / 255);
      break;
    default:
      break;
  }
}

void onI2CRequest() {
  Wire1.write((uint8_t)tx_mode);
  Wire1.write((uint8_t)source);
}

void setup1() {
  Wire1.setSDA(I2C_SDA);
  Wire1.setSCL(I2C_SCL);
  Wire1.begin(I2C_ADDR);
  Wire1.onReceive(onI2CReceive);
  Wire1.onRequest(onI2CRequest);
}

void loop1() {
  tight_loop_contents();
}
