/* VNA

*/

/*
 * Copyright (c) 2018 Daniel Marks

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */

#include <Wire.h>
#include "VNA.h"
#include "si5351.h"
#include "tinycl.h"
#include "debugmsg.h"
#include "complex.h"
#include "flashstruct.h"
#include "mini-printf.h"

#ifdef VNA_TOUCHSCREEN
#include "touchscreen.h"
#endif

#define IFCLOCK_PIN PB15
#define IFCLOCK_PIN_CTR PA15
#define IFCLOCK_PORT2_PIN PA8
#define AD_CUR_PIN PA0
#define AD_VOLT_PIN PB0
#define AD_CUR2_PIN PA2
#define ATTEN_PIN PB14
#define TOUCHSCREEN_DISABLE_PIN PB13

#ifdef VNA_TOUCHSCREEN
bool touchscreen_enabled;
#endif


Si5351 si5351;

int pinMapADCCURPINin;
int pinMapADCVOLTPINin;
int pinMapADCCUR2PINin;

const vna_flash_header flash_header = { 0xDEADBEEF, 0xC001ACE5 };
vna_acquisition_state vna_state = {3000000u, 30000000u, 50u, 64, 1000, VNA_MAX_FREQS, 0, 1, 0, 0, VNA_NO_CALIB };
vna_calib_oneport *vna_1pt = NULL;
vna_calib_freq_parm vna_calib[VNA_MAX_FREQS];

void setup_analog(int accurate)
{
  pinMapADCCURPINin = PIN_MAP[AD_CUR_PIN].adc_channel;
  pinMapADCVOLTPINin = PIN_MAP[AD_VOLT_PIN].adc_channel;
  pinMapADCCUR2PINin = PIN_MAP[AD_CUR2_PIN].adc_channel;
  adc_set_sample_rate(ADC1, accurate ? ADC_SMPR_13_5 : ADC_SMPR_1_5);
  adc_set_reg_seqlen(ADC1, 1);
  adc_set_sample_rate(ADC2, accurate ? ADC_SMPR_13_5 : ADC_SMPR_1_5);
  adc_set_reg_seqlen(ADC2, 1);
}

volatile short sampCUR, sampVOLT, sampCUR2;

volatile int number_to_sum = 256;
volatile int current_summed = 0;

volatile int sampCURI, sampVOLTI, sampCUR2I, sampPWR;
volatile int sampCURQ, sampVOLTQ, sampCUR2Q;

unsigned short analogReadPins(void)
{
  ADC1->regs->SQR3 = pinMapADCCURPINin;
  ADC1->regs->CR2 |= ADC_CR2_SWSTART;
  ADC2->regs->SQR3 = pinMapADCVOLTPINin;
  ADC2->regs->CR2 |= ADC_CR2_SWSTART;

  while (!(ADC1->regs->SR & ADC_SR_EOC));
  sampCUR = (ADC1->regs->DR & ADC_DR_DATA);

  ADC1->regs->SQR3 = pinMapADCCUR2PINin;
  ADC1->regs->CR2 |= ADC_CR2_SWSTART;

  while (!(ADC2->regs->SR & ADC_SR_EOC));
  sampVOLT = (ADC2->regs->DR & ADC_DR_DATA);

  while (!(ADC1->regs->SR & ADC_SR_EOC));
  sampCUR2 = (ADC1->regs->DR & ADC_DR_DATA);
}

void setup_pins(void)
{
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY); // release PB3 and PB5
  afio_remap(AFIO_REMAP_SPI1); // remap SPI1

  gpio_set_mode(GPIOB, 3, GPIO_AF_OUTPUT_PP);
  gpio_set_mode(GPIOB, 4, GPIO_INPUT_FLOATING);
  gpio_set_mode(GPIOB, 5, GPIO_AF_OUTPUT_PP);

  pinMode(ATTEN_PIN, OUTPUT);
  digitalWrite(ATTEN_PIN, LOW);
  pinMode(IFCLOCK_PIN, INPUT);
  pinMode(IFCLOCK_PIN_CTR, INPUT);
  pinMode(IFCLOCK_PORT2_PIN, INPUT);
  pinMode(AD_CUR_PIN, INPUT_ANALOG);
  pinMode(AD_VOLT_PIN, INPUT_ANALOG);
  pinMode(AD_CUR2_PIN, INPUT_ANALOG);
#ifdef VNA_TOUCHSCREEN
  pinMode(TOUCHSCREEN_DISABLE_PIN, INPUT_PULLUP);
  delay(1);
  touchscreen_enabled = digitalRead(TOUCHSCREEN_DISABLE_PIN) == HIGH;
#endif
}

#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT
#define DEMCR_TRCENA    0x01000000

static void initialize_cortex_m3_cycle_counter(void)
{
  DEMCR |= DEMCR_TRCENA;
  *DWT_CYCCNT = 0;
  DWT_CTRL |= CYCCNTENA;
}

volatile unsigned int diftick;
volatile unsigned int adclocks;
volatile unsigned int lasttick = 0;

unsigned int numphases = 4;
volatile unsigned int curphase = 0;

//unsigned int timephase[30];

void ifClockInterrupt(void)
{
  unsigned int timerval = CPU_CYCLES;

  diftick = ((unsigned int)(timerval - lasttick));
  lasttick = timerval;

  if (current_summed < number_to_sum)
  {
    if (current_summed == 0)
    {
      sampCURI = 0; sampCURQ = 0;
      sampVOLTI = 0; sampVOLTQ = 0;
      sampCUR2I = 0; sampCUR2Q = 0;
      sampPWR = 0;
    }
    current_summed++;
    //timephase[0] = timerval;
    timer_pause(TIMER3);
    timer_set_count(TIMER3, 0);
    timerval = diftick / numphases;
    if (timerval > (F_CPU / 200000u))
    {
      timer_set_reload(TIMER3, timerval);
      //    timer_set_compare(TIMER3,1,0);
      timer_generate_update(TIMER3);
      curphase = 0;
      timer_resume(TIMER3);
    }
  }
}

void timerContInterrupt(void)
{
  //unsigned int timerval = CPU_CYCLES;

  if (curphase < numphases)
  {
    //timephase[curphase] = timerval;
    analogReadPins();
    switch (curphase & 0x03)
    {
      case 0: sampCURI += sampCUR;
        sampVOLTI += sampVOLT;
        sampCUR2I += sampCUR2;
        break;
      case 1: sampCURQ -= sampCUR;
        sampVOLTQ -= sampVOLT;
        sampCUR2Q -= sampCUR2;
        break;
      case 2: sampCURI -= sampCUR;
        sampVOLTI -= sampVOLT;
        sampCUR2I -= sampCUR2;
        break;
      case 3: sampCURQ += sampCUR;
        sampVOLTQ += sampVOLT;
        sampCUR2Q += sampCUR2;
        break;
    }
    sampPWR += sampCUR2;
    curphase++;
  }
}

void setup_if_clock(void)
{
  nvic_irq_set_priority(NVIC_EXTI_15_10, 2);
  attachInterrupt(IFCLOCK_PIN, ifClockInterrupt, RISING);

  nvic_irq_set_priority(NVIC_TIMER3, 2);
  timer_init(TIMER3);
  timer_attach_interrupt(TIMER3, 1, timerContInterrupt);
}


void setup() {
  initialize_cortex_m3_cycle_counter();
  setup_pins();

  vna_initialize_si5351();

#ifdef VNA_TOUCHSCREEN
  tcal.iscal = false;
  if (touchscreen_enabled) touchscreen_setup();
#endif

  setup_if_clock();
  setup_analog(0);
  vna_readcal(1);
}

void reset_calib_state(void)
{
  if (vna_1pt != NULL) free(vna_1pt);
  vna_1pt = NULL;
  vna_state.calib_state = VNA_NO_CALIB;
}

void free_calib_memory(void)
{
  if (vna_1pt != NULL) free(vna_1pt);
  vna_1pt = NULL;
  vna_state.calib_state &= ~VNA_ALL_1PT;
}

int acquire_sample(unsigned int averages, unsigned int timeout)
{
  bool timedout = false;
  unsigned int inittime, curtime;

  //timer_init(TIMER3);
  timer_pause(TIMER3);
  timer_attach_interrupt(TIMER3, 1, timerContInterrupt);
  timer_set_prescaler(TIMER3, 0);
  timer_set_mode(TIMER3, 1, TIMER_OUTPUT_COMPARE);
  timer_set_count(TIMER3, 0);
  timer_set_reload(TIMER3,65535);
  timer_set_compare(TIMER3, 1, 0);
  timer_generate_update(TIMER3);

  timeout = timeout * (F_CPU / 1000u);
  /* wait for previous acquisition to stop */
  inittime = CPU_CYCLES;
  timedout = false;
  current_summed = 0;
  while (current_summed < number_to_sum)
  {
    curtime = CPU_CYCLES;
    if (((unsigned int)(curtime - inittime)) > timeout)
    {
      timedout = true;
      break;
    }
  }
  if (timedout) return -1;
  number_to_sum = averages;
  current_summed = 0;
  /* wait for previous acquisition to stop */
  inittime = CPU_CYCLES;
  timedout = false;
  while (current_summed < number_to_sum)
  {
    curtime = CPU_CYCLES;
    if (((unsigned int)(curtime - inittime)) > timeout)
    {
      timedout = true;
      break;
    }
  }
  if (timedout) return -1;
  return 0;
}

int setup_frequency_acquire(unsigned int frequency)
{
  if ((frequency < VNA_MIN_FREQ) || (frequency > VNA_MAX_FREQ))
    return -1;

  rcc_clk_enable(RCC_I2C1);
  si5351.output_enable(SI5351_CLK0, 0);
  si5351.output_enable(SI5351_CLK1, 0); 
  if (frequency < VNA_FREQ_3X)
  {
    si5351.set_freq(frequency * SI5351_FREQ_MULT, SI5351_CLK0);
    si5351.set_freq((frequency + VNA_NOMINAL_1X_IF_FREQ) * SI5351_FREQ_MULT, SI5351_CLK1);
    numphases = 4;
  } else
  {
    frequency = frequency / 3;
    si5351.set_freq(frequency * SI5351_FREQ_MULT, SI5351_CLK0);
    si5351.set_freq((frequency + VNA_NOMINAL_3X_IF_FREQ) * SI5351_FREQ_MULT, SI5351_CLK1);
    numphases = 12;
  }
  delay(2);
  si5351.output_enable(SI5351_CLK0, 1);
  si5351.output_enable(SI5351_CLK1, 1); 
  rcc_clk_disable(RCC_I2C1);
  digitalWrite(ATTEN_PIN, vna_state.atten ? HIGH : LOW);
  return 0;
}

int vna_display_dataset_operation(vna_acquire_dataset_state *vads, void *va)
{
  if (vna_state.csv)
  {
    Serial.print(vads->freq);
    Serial.print(",");
    Serial.print(vads->volti);
    Serial.print(",");
    Serial.print(vads->voltq);
    Serial.print(",");
    Serial.print(vads->curi);
    Serial.print(",");
    Serial.print(vads->curq);
    Serial.print(",");
    Serial.print(vads->cur2i);
    Serial.print(",");
    Serial.println(vads->cur2q);
  } else
  {
    Serial.print("Freq ");
    Serial.print(vads->freq);
    Serial.print(": Volt (");
    Serial.print(vads->volti);
    Serial.print(",");
    Serial.print(vads->voltq);
    Serial.print(") Cur1 (");
    Serial.print(vads->curi);
    Serial.print(",");
    Serial.print(vads->curq);
    Serial.print(") Cur2 (");
    Serial.print(vads->cur2i);
    Serial.print(",");
    Serial.print(vads->cur2q);
    Serial.println(")");
  }
}

void vna_initialize_si5351()
{
  rcc_clk_enable(RCC_I2C1);
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 27000000u, 0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_6MA);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_6MA);
  si5351.set_ms_source(SI5351_CLK0, SI5351_PLLA);
  si5351.set_ms_source(SI5351_CLK1, SI5351_PLLB);
  //si5351.set_freq(10000000ull * SI5351_FREQ_MULT, SI5351_CLK0);
  //si5351.set_freq(10010000ull * SI5351_FREQ_MULT, SI5351_CLK1);
  //si5351.output_enable(SI5351_CLK0, 1);
  //si5351.output_enable(SI5351_CLK1, 1); 
  rcc_clk_disable(RCC_I2C1);
}

bool vna_acquire_dataset(vna_acquisition_state *vs, vna_acquire_dataset_operation vado, void *v)
{
  int n;
  vna_acquire_dataset_state vads;

  vna_initialize_si5351();
  unsigned int freqstep = (vs->endfreq - vs->startfreq) / vs->nfreqs;
  for (n = 0; n < vs->nfreqs; n++)
  {
    unsigned int freq = vs->startfreq + freqstep * n;
    setup_frequency_acquire(freq);
    if (n == 0)
    {
      if (acquire_sample(12, vs->timeout) < 0)
        return false;
    }
    if (acquire_sample(vs->num_averages, vs->timeout) < 0)
      return false;
    vads.n = n;
    vads.total = vs->nfreqs;
    vads.freq = freq;
    vads.volti = sampVOLTI;
    vads.voltq = sampVOLTQ;
    vads.curi = sampCURI;
    vads.curq = sampCURQ;
    vads.cur2i = sampCUR2I;
    vads.cur2q = sampCUR2Q; 
    vado(&vads, v);
    if (Serial.available())
    {
      if (Serial.read() == '!')
        return false;
    }
#ifdef VNA_TOUCHSCREEN
    if (touchscreen_enabled && touchscreen_abort())
       return false;
#endif
  }
  return 1;
}

int vna_set_averages(unsigned short averages, unsigned short timeout)
{
  if ((averages >= 1) && (averages <= 5000))  
     vna_state.num_averages = averages;
  if ((timeout >= 1) && (timeout <= 10000))
     vna_state.timeout = timeout < 500 ? 500 : timeout;
  return 1;
}

int vna_set_characteristic_impedance(unsigned int char_impedance)
{
  if ((char_impedance >= 1) && (char_impedance <= 100000u))
      vna_state.char_impedance = char_impedance;
  return 1;
}

int averages_cmd(int args, tinycl_parameter* tp, void *v)
{
  vna_set_averages(tp[0].ti.i,tp[1].ti.i);
  Serial.print("Averages=");
  Serial.println(vna_state.num_averages);
  Serial.print("Timeout=");
  Serial.println(vna_state.timeout);
  return 1;
}

int csv_cmd(int args, tinycl_parameter* tp, void *v)
{
  int n;
  vna_acquisition_state *vs = &vna_state;
  vs->csv = tp[0].ti.i;
  Serial.print("CSV=");
  Serial.println(vs->csv);
  return 1;
}

int debug_cmd(int args, tinycl_parameter* tp, void *v)
{
  int n;
  vna_acquisition_state *vs = &vna_state;
  setDebugMsgMode(tp[0].ti.i);
  Serial.print("DEBUG=");
  Serial.println(debugmsg_state);
  return 1;
}

const char *atten_strings[3] = { "0 Off", "1 On" };

int atten_cmd(int args, tinycl_parameter* tp, void *v)
{
  int n = tp[0].ti.i;

  if ((n < 0) || (n > 1))
  {
    Serial.println("Invalid setting");
    return 1;
  }
  vna_state.atten = n;
  Serial.print("atten=");
  Serial.println(atten_strings[vna_state.atten]);
  return 1;
}

const char *series_shunt_strings[3] = { "0 Series", "1 Shunt" };

int shunt_cmd(int args, tinycl_parameter* tp, void *v)
{
  int n = tp[0].ti.i;

  if ((n < 0) || (n > 1))
  {
    Serial.println("Invalid setting");
    return 1;
  }
  vna_state.series_shunt_two = n;
  Serial.print("impedance-mode=");
  Serial.println(series_shunt_strings[vna_state.series_shunt_two]);
  return 1;
}

int doacq_cmd(int args, tinycl_parameter* tp, void *v)
{
  int n;
  vna_acquisition_state *vs = &vna_state;
  vna_acquire_dataset(vs, vna_display_dataset_operation, NULL);
  return 1;
}

int vna_set_frequencies(unsigned int nfreqs, unsigned int startfreq, unsigned int endfreq)
{
  if ((startfreq >= VNA_MIN_FREQ) && (endfreq > startfreq) && (endfreq <= VNA_MAX_FREQ))
  {
    vna_state.startfreq = startfreq;
    vna_state.endfreq = endfreq;
  } else return 0;
  if ((nfreqs >= 1) && (nfreqs <= VNA_MAX_FREQS)) vna_state.nfreqs = nfreqs;
    else return 0;
  reset_calib_state();
  return 1;
}

int setacq_cmd(int args, tinycl_parameter* tp, void *v)
{
  unsigned int nfreqs = tp[0].ti.i;
  unsigned int startfreq = tp[1].ti.i;
  unsigned int endfreq = tp[2].ti.i;
  bool statereset = (vna_state.calib_state != VNA_NO_CALIB);
  if (!vna_set_frequencies(nfreqs, startfreq, endfreq))
  {
      Serial.println("Invalid Parameters");
      return 1;
  }
  Serial.print("Start Frequency = ");
  Serial.println(vna_state.startfreq);
  Serial.print("End Frequency = ");
  Serial.println(vna_state.endfreq);
  Serial.print("Number Frequencies = ");
  Serial.println(vna_state.nfreqs);
  if (statereset)
    Serial.println("Calibration state reset");
  return 1;
}

int imacq_cmd(int args, tinycl_parameter* tp, void *v)
{
  vna_acquisition_state vs = vna_state;
  vs.nfreqs = tp[0].ti.i;
  vs.startfreq = tp[1].ti.i;
  vs.endfreq = tp[2].ti.i;
  if (vs.nfreqs < 1)
  {
    Serial.println("Invalid number of frequencies");
    return 1;
  }
  if ((vs.startfreq < VNA_MIN_FREQ) || (vs.endfreq < vs.startfreq) && (vs.endfreq > VNA_MAX_FREQ))
  {
    Serial.println("Invalid maximum or minimum frequency");
    return 1;
  }
  vna_acquire_dataset(&vs, vna_display_dataset_operation, NULL);
  return 1;
}

int vna_open_dataset_operation(vna_acquire_dataset_state *vads, void *va)
{
  vna_calib_oneport *v1pt = (vna_calib_oneport *)va;
  v1pt[vads->n].zo = Complex((float)vads->volti, (float)vads->voltq) / Complex((float)vads->curi, (float)vads->curq);
  if (debugmsg_state)
  {
    char s[80];
    mini_snprintf(s,sizeof(s)-1,"Open freq %u zo=%06f + j%06f",vads->freq, float2int32(v1pt[vads->n].zo.real), float2int32(v1pt[vads->n].zo.imag));
    Serial.println(s);
  }
}

int vna_short_dataset_operation(vna_acquire_dataset_state *vads, void *va)
{
  vna_calib_oneport *v1pt = (vna_calib_oneport *)va;
  v1pt[vads->n].zs = Complex((float)vads->volti, (float)vads->voltq) / Complex((float)vads->curi, (float)vads->curq);
  vna_calib[vads->n].s2 = Complex((float)vads->cur2i,(float)vads->cur2q) / ((float) vna_state.num_averages);
  if (debugmsg_state)
  {
    char s[80];
    mini_snprintf(s,sizeof(s)-1,"Short freq %u zs=%06f + j%06f",vads->freq, float2int32(v1pt[vads->n].zs.real), float2int32(v1pt[vads->n].zs.imag));
    Serial.println(s);
  }
}

int vna_load_dataset_operation(vna_acquire_dataset_state *vads, void *va)
{
  vna_calib_oneport *v1pt = (vna_calib_oneport *)va;
  v1pt[vads->n].zt = Complex((float)vads->volti, (float)vads->voltq) / Complex((float)vads->curi, (float)vads->curq);
  if (debugmsg_state)
  {
    char s[80];
    mini_snprintf(s,sizeof(s)-1,"Load freq %u zt=%06f + j%06f",vads->freq, float2int32(v1pt[vads->n].zt.real), float2int32(v1pt[vads->n].zt.imag));
    Serial.println(s);
  }
}

int vna_allocate_1pt_calib(void)
{
  if (vna_1pt != NULL) return 1;
  if ((vna_1pt = (vna_calib_oneport *) malloc(sizeof(vna_calib_oneport) * vna_state.nfreqs)) == NULL)
  {
    vna_state.calib_state = VNA_NO_CALIB;
    return 0;
  }
  return 1;
}

int vna_complete_1pt_calib(void)
{
  int n;

  if ((vna_state.calib_state & VNA_ALL_1PT) != VNA_ALL_1PT)
    return 0;
  if (vna_1pt == NULL) return 0;
  float z0 = vna_state.char_impedance;
  for (n = 0; n < vna_state.nfreqs; n++)
  {
    vna_calib_freq_parm *vc = &vna_calib[n];
    vc->b = -vna_1pt[n].zs;
    vc->c = (vna_1pt[n].zs - vna_1pt[n].zt) / ((vna_1pt[n].zo - vna_1pt[n].zt) * z0);
    vc->d = (-vna_1pt[n].zo) * vc->c;
  }
  vna_state.calib_state |= VNA_VALID_CALIB_1PT;
  vna_state.calib_state &= ~VNA_VALID_CALIB_2PT;
  free_calib_memory();
  return 1;
}

int vna_opencalib(void)
{
  if (!vna_allocate_1pt_calib()) return 0;
  if (!vna_acquire_dataset(&vna_state, vna_open_dataset_operation, (void *)vna_1pt)) return 0;
  vna_state.calib_state |= VNA_OPEN_CALIB;
  return vna_complete_1pt_calib() ? 2 : 1;
}

int opencalib_cmd(int args, tinycl_parameter* tp, void *v)
{
  Serial.println("Open calibration");
  switch (vna_opencalib())
  {
    case 0: Serial.println("Calibration aborted");
            return 1;
            break;
    case 1: Serial.println("\r\nOpen calibration successful");
            break;
    case 2: Serial.println("\r\n1 Port calibration successful");
            break;
  }
  return 1;
}

int vna_shortcalib(void)
{
  if (!vna_allocate_1pt_calib()) return 0;
  if (!vna_acquire_dataset(&vna_state, vna_short_dataset_operation, (void *)vna_1pt)) return 0;
  vna_state.calib_state |= VNA_SHORT_CALIB;
  return vna_complete_1pt_calib() ? 2 : 1;
}

int shortcalib_cmd(int args, tinycl_parameter* tp, void *v)
{
  Serial.println("Short calibration");
  switch (vna_shortcalib())
  {
    case 0: Serial.println("Calibration aborted");
            return 1;
            break;
    case 1: Serial.println("\r\nShort calibration successful");
            break;
    case 2: Serial.println("\r\n1 Port calibration successful");
            break;
  }
  return 1;
}

int vna_loadcalib(void)
{
  if (!vna_allocate_1pt_calib()) return 0;
  if (!vna_acquire_dataset(&vna_state, vna_load_dataset_operation, (void *)vna_1pt)) return 0;
  vna_state.calib_state |= VNA_LOAD_CALIB;
  return vna_complete_1pt_calib() ? 2 : 1;
}

int loadcalib_cmd(int args, tinycl_parameter* tp, void *v)
{
  Serial.println("Load calibration");
  vna_set_characteristic_impedance(tp[0].ti.i);
  switch (vna_loadcalib())
  {
    case 0: Serial.println("Calibration aborted");
            return 1;
            break;
    case 1: Serial.println("\r\nLoad calibration successful");
            break;
    case 2: Serial.println("\r\n1 Port calibration successful");
            break;
  }
  return 1;
}

int vna_twocalib_dataset_operation(vna_acquire_dataset_state *vads, void *va)
{
  Complex voltpt((float)vads->volti, (float)vads->voltq);
  Complex curpt((float)vads->curi, (float)vads->curq);
  Complex cur2pt((float)vads->cur2i, (float)vads->cur2q);
  vna_calib_freq_parm *vc = &vna_calib[vads->n];
  Complex vn = voltpt + curpt * vc->b;
  Complex in = curpt * vc->d + voltpt * vc->c;
  cur2pt = cur2pt - vc->s2*((float)vna_state.num_averages);
  vc->z2 = vn / cur2pt;
  vc->i2 = in / cur2pt;
  if (debugmsg_state)
  {
    char s[80];
    mini_snprintf(s,sizeof(s)-1,"Thru freq %u z2=%06f + j%06f  i2 = %06f + j%06f",vads->freq, float2int32(vc->z2.real),float2int32(vc->z2.imag),float2int32(vc->i2.real),float2int32(vc->i2.imag));
    Serial.println(s);
  }
}

int vna_thrucal(void)
{
  if (!(vna_state.calib_state & VNA_VALID_CALIB_1PT)) return 0;
  if (!vna_acquire_dataset(&vna_state, vna_twocalib_dataset_operation, NULL))
  {
    vna_state.calib_state &= ~VNA_VALID_CALIB_2PT;
    return 1;
  }
  vna_state.calib_state |= VNA_VALID_CALIB_2PT;
  return 2;
}

int twocalib_cmd(int args, tinycl_parameter* tp, void *v)
{
  Serial.println("Thru calibration");
  switch (vna_thrucal())
  {
    case 0: Serial.println("Two Port Must Have Valid One Port Cal");
            break;
    case 1: Serial.println("Calibration aborted");
            break;
    case 2:  Serial.println("\r\nTwo port calibration successful");
            break;
  }
  return 1;
}

int vna_rtr_impedance_display(int n, int total, unsigned int freq, bool ch2, Complex imp, Complex zthru)
{
  char s[80];
  mini_snprintf(s,sizeof(s)-1,vna_state.csv ? "%u,%04f,%04f" : "Freq: %u Z (%04f,%04f)",freq,float2int32(imp.real),float2int32(imp.imag));
  Serial.print(s);
  if (ch2)
  {
    mini_snprintf(s,sizeof(s)-1,vna_state.csv ? ",%04f,%04f" : " ZT (%04f,%04f)",float2int32(zthru.real),float2int32(zthru.imag));
    Serial.print(s);
  }
  Serial.println("");
}

int vna_display_acq_operation(vna_acquire_dataset_state *vads, void *va)
{
  Complex voltpt((float)vads->volti, (float)vads->voltq);
  Complex curpt((float)vads->curi, (float)vads->curq);
  vna_calib_freq_parm *vc = &vna_calib[vads->n];
  Complex vn = voltpt + curpt * vc->b;
  Complex in = curpt * vc->d + voltpt * vc->c;
  Complex imp = vn / in;
  Complex zthru;
  
  if (vna_state.calib_state & VNA_VALID_CALIB_2PT)
  {
    Complex curpt2((float)vads->cur2i, (float)vads->cur2q);
    curpt2 = curpt2 - vc->s2 * ((float)vna_state.num_averages);
    if (vna_state.series_shunt_two)
    {
       zthru = (curpt2 * vc->z2) / (in - curpt2 * vc->i2);     
    } else
    {
      zthru = (vn - curpt2 * vc->z2) / (curpt2 * vc->i2);
    }
  }
  ((vna_report_trans_reflected)va)(vads->n,vads->total,vads->freq,(vna_state.calib_state & VNA_VALID_CALIB_2PT) != 0,imp.conj(),zthru.conj());
}

//   another way to calculate shunt/series impedance
//    float z0 = (float)vna_state.char_impedance;
//    Complex s21;
//    s21 = curpt2 * (vc->z2 + vc->i2 * z0) / (vn + in * z0);
//        zthru = (s21/(s21 - 1.0f)) * z0 * (-0.5f);  shunt
//        zthru = ((s21 - 1.0f)/s21) * z0 * (-2.0f);  series

int vna_acquire_impedance(vna_report_trans_reflected vrtr)
{
  if (!(vna_state.calib_state & VNA_VALID_CALIB_1PT)) return 0;
  if (!vna_acquire_dataset(&vna_state, vna_display_acq_operation, (void *)vrtr)) return 1;
  return 2;  
}

int acq_cmd(int args, tinycl_parameter* tp, void *v)
{
  switch (vna_acquire_impedance(vna_rtr_impedance_display))
  {
    case 0: Serial.println("Can only be performed after 1 or 2 port calibration");
            break;
    case 1: Serial.println("Acquisition aborted");
            break;
    case 2: Serial.println("-----");
            break;
  }
  return 1;
}

int vna_rtr_sparm_display(int n, int total, unsigned int freq, bool ch2, Complex s11, Complex s21)
{
  float s11db = (10.0f / logf(10.0f)) * logf(s11.absq());
  float s11deg = RAD2DEG(s11.arg());
  float s21db, s21deg;
  char s[80];
  if (ch2)
  {
    s21db = (10.0f / logf(10.0f)) * logf(s21.absq());
    s21deg = RAD2DEG(s21.arg());    
  }
  mini_snprintf(s,sizeof(s)-1,vna_state.csv ? "%u,%04f,%04f" : "Freq %u: S11 (%04f,%04f)",freq,float2int32(s11db),float2int32(s11deg));
  Serial.print(s);
  if (ch2)
  {
    mini_snprintf(s,sizeof(s)-1," S21 (%04f,%04f)",float2int32(s21db),float2int32(s21deg));
    Serial.print(s);
  }
  Serial.println("");
}

int vna_display_sparm_operation(vna_acquire_dataset_state *vads, void *va)
{
  Complex voltpt((float)vads->volti, (float)vads->voltq);
  Complex curpt((float)vads->curi, (float)vads->curq);
  vna_calib_freq_parm *vc = &vna_calib[vads->n];
  Complex vn = voltpt + curpt * vc->b;
  Complex in = curpt * vc->d + voltpt * vc->c;
  Complex imp = vn / in;
  float z0 = (float)vna_state.char_impedance;
  Complex s11 = (imp - z0) / (imp + z0);
  Complex s21;
  if (vna_state.calib_state & VNA_VALID_CALIB_2PT)
  {
    Complex curpt2((float)vads->cur2i, (float)vads->cur2q);
    curpt2 = curpt2 - vc->s2 * ((float)vna_state.num_averages);
    s21 = curpt2 * (vc->z2 + vc->i2 * z0) / (vn + in * z0);
  }
  ((vna_report_trans_reflected)va)(vads->n,vads->total,vads->freq,(vna_state.calib_state & VNA_VALID_CALIB_2PT) != 0,s11.conj(),s21.conj());
}

int vna_acquire_sparm(vna_report_trans_reflected vrtr)
{
  if (!(vna_state.calib_state & VNA_VALID_CALIB_1PT)) return 0;
  if (!vna_acquire_dataset(&vna_state, vna_display_sparm_operation, (void *)vrtr)) return 1;  
  return 2;
}

int sparm_cmd(int args, tinycl_parameter* tp, void *v)
{
  switch (vna_acquire_sparm(vna_rtr_sparm_display))
  {
    case 0: Serial.println("Can only be performed after 1 or 2 port calibration");
            break;
    case 1: Serial.println("Acquisition aborted");
            break;
    case 2: Serial.println("-----");
            break;
  }
  return 1;
}

const unsigned int flash_pages[] = { 0x08018000u, 0x08019000u, 0x0801A000u, 0x0801B000u, 0x0801C000u, 0x0801D000u, 0x0801E000u, 0x0801F000u };
#define NUM_FLASH_PAGES ((sizeof(flash_pages)/sizeof(void *)))

int vna_writecal(int n)
{
  void *vp[4];
  int b[4];

  if ((n < 1) || (n > NUM_FLASH_PAGES)) return 0;
  vp[0] = (void *)&flash_header;
  b[0] = sizeof(flash_header);
  vp[1] = &vna_state;
  b[1] = sizeof(vna_state);
  vp[2] = vna_calib;
  b[2] = sizeof(vna_calib);
#ifdef VNA_TOUCHSCREEN
  vp[3] = &tcal;
  b[3] = sizeof(tcal);
  return writeflashstruct((void *)flash_pages[n - 1], 4, vp, b);
#else
  return writeflashstruct((void *)flash_pages[n - 1], 3, vp, b);
#endif
}

int writecal_cmd(int args, tinycl_parameter* tp, void *v)
{
  int n = tp[0].ti.i;
  if (!(vna_state.calib_state & VNA_VALID_CALIB_1PT))
  {
    Serial.println("There is no calibration to write");
    return -1;
  }
  if ((n < 1) || (n > NUM_FLASH_PAGES))
  {
    Serial.println("Invalid calibration number");
    return 1;
  }
  Serial.print("Writing calibration ");
  Serial.print(n);
  Serial.println(vna_writecal(n) ? ": written" : ": failed");
  return 1;
}

int vna_readcal(int n)
{
  int err;
  vna_flash_header rd_flash_header;
  void *vp[4];
  int b[4];
  if ((n < 1) || (n > NUM_FLASH_PAGES)) return 0;

  vp[0] = &rd_flash_header;
  b[0] = sizeof(rd_flash_header);
  readflashstruct((void *)flash_pages[n - 1], 1, vp, b);

  if (!((rd_flash_header.flash_header_1 == flash_header.flash_header_1) && (rd_flash_header.flash_header_2 == flash_header.flash_header_2)))
    return 0;
    
  vp[0] = NULL;
  b[0] = sizeof(flash_header);
  vp[1] = &vna_state;
  b[1] = sizeof(vna_state);
  vp[2] = vna_calib;
  b[2] = sizeof(vna_calib);
#ifdef VNA_TOUCHSCREEN
  vp[3] = &tcal;
  b[3] = sizeof(tcal);
  err = readflashstruct((void *)flash_pages[n - 1], 4, vp, b);
#else
  err = readflashstruct((void *)flash_pages[n - 1], 3, vp, b);
#endif
  free_calib_memory();
  return err;
}

int readcal_cmd(int args, tinycl_parameter* tp, void *v)
{
  int n = tp[0].ti.i;
 
  if ((n < 1) || (n > NUM_FLASH_PAGES))
  {
    Serial.println("Invalid calibration number");
    return 1;
  }
  if (vna_readcal(n))
  {
    Serial.print("Read calibration ");
    Serial.println(n);
  } else
    Serial.println("No calibration to retrieve");
  return 1;
}

int vna_calentry(int n, char *s, int len)
{
  vna_flash_header rd_flash_header;
  vna_acquisition_state rd_vna_state;
  char splitchar = (len < 0) ? '\n' : ' ';
  
  if (len < 0) len = -len;
  if ((n < 1) || (n > NUM_FLASH_PAGES)) return 0;
  
  void *vp[2];
  int b[2];
  vp[0] = &rd_flash_header;
  b[0] = sizeof(rd_flash_header);
  vp[1] = &rd_vna_state;
  b[1] = sizeof(rd_vna_state);
  if (readflashstruct((void *)flash_pages[n-1], 2, vp, b))
  {
    if ((rd_flash_header.flash_header_1 == flash_header.flash_header_1) && (rd_flash_header.flash_header_2 == flash_header.flash_header_2))
    {
      mini_snprintf(s,len,"%d: Start %u End %u%c# %u Impedance %u Avgs %u %s",
         n,
         rd_vna_state.startfreq,
         rd_vna_state.endfreq,
         splitchar,
         rd_vna_state.nfreqs,
         rd_vna_state.char_impedance,
         rd_vna_state.num_averages,
         (rd_vna_state.calib_state & VNA_VALID_CALIB_2PT) ? "2 Port" : 
            ((rd_vna_state.calib_state & VNA_VALID_CALIB_1PT) ? "1 Port" : "No Cal"));
       return 1;
    } else
    {
       mini_snprintf(s,len,"%d: No Calibration",n);
    }
  }
  return 0;
}

int listcal_cmd(int args, tinycl_parameter* tp, void *v)
{
  int n;
  char s[120];
  Serial.println("Calibrations:");
  for (n = 1; n <= NUM_FLASH_PAGES; n++)
  {
    vna_calentry(n,s,sizeof(s)-1);
    Serial.println(s);
  }
  Serial.println("-----");
  return 1;
}

unsigned int get_number_serial()
{
  unsigned int num = 0;

  for (;;)
  {
    int c;
    while ((c = Serial.read()) == -1);
    if ((c >= '0') && (c <= '9')) num = num*10 + c - '0';
    if (c == '\r') break;
  }
  return num;
}

const uint8 four_blanks[4] = {0xFF,0x03,0xFF,0x03};

unsigned char vna_rtr_remote_cmd_port;
unsigned short vna_rtr_remote_cmd_samples;

int vna_rtr_remote_cmd(int n, int total, unsigned int freq, bool ch2, Complex s11, Complex s21)
{
  uint8_t b[4];
  float db, deg;
  if (vna_rtr_remote_cmd_samples >= total)
    return 0;
  if (vna_rtr_remote_cmd_port == 1) s11 = s21;
  db = (10.0f / logf(10.0f)) * logf(s11.absq());
  deg = RAD2DEG(s11.arg());
  short dbint = 344.0f-db*17.0f;
  short degint = (deg < 0.0f ? 360.0f+deg : deg)*(1024.0f/180.0f);
  dbint = (dbint < 0) ? 0 : dbint;
  degint = (degint < 5) ? 5 : degint;
  degint = (degint > 2042) ? 2042 : degint;
  b[0] = degint & 0xFF;
  b[1] = degint >> 8;
  b[2] = dbint & 0xFF;
  b[3] = dbint >> 8;
  Serial.write((const uint8_t *)b,sizeof(b));
  vna_rtr_remote_cmd_samples++;
}

#define VNA_FREQUENCY_TUNING_CONVERSION_NUMERATOR   ((uint64_t)100000000)
#define VNA_FREQUENCY_TUNING_CONVERSION_DENOMINATOR ((uint64_t)1073741824)

// huge hack, but the minivna is an old antiquated thing

void remote_cmd(int port)
{ 
   int i;
   tinycl_do_echo = 0;
   vna_rtr_remote_cmd_samples = 0;
   unsigned int frequency_tuning_word, number_of_samples, frequency_tuning_step;
   vna_rtr_remote_cmd_port = port;
   frequency_tuning_word = get_number_serial();
   number_of_samples = get_number_serial();
   frequency_tuning_step = get_number_serial();
   uint32_t start_frequency = (((uint64_t)frequency_tuning_word)*VNA_FREQUENCY_TUNING_CONVERSION_NUMERATOR)/VNA_FREQUENCY_TUNING_CONVERSION_DENOMINATOR;
   uint32_t step_frequency = (((uint64_t)frequency_tuning_step)*VNA_FREQUENCY_TUNING_CONVERSION_NUMERATOR)/VNA_FREQUENCY_TUNING_CONVERSION_DENOMINATOR;
   uint32_t stop_frequency = start_frequency + number_of_samples * step_frequency;
   if ((start_frequency != vna_state.startfreq) || (stop_frequency != vna_state.endfreq) || (number_of_samples != vna_state.nfreqs))
      vna_set_frequencies(number_of_samples, start_frequency, stop_frequency); 
   vna_acquire_sparm(vna_rtr_remote_cmd);
   for (i=vna_rtr_remote_cmd_samples;i<number_of_samples;i++)
      Serial.write((const uint8 *)four_blanks,sizeof(four_blanks));
}

int remote_0_cmd(int args, tinycl_parameter* tp, void *v)
{
  remote_cmd(0);
  return 0;
}

int remote_1_cmd(int args, tinycl_parameter* tp, void *v)
{
  remote_cmd(1);
  return 0;
}

const tinycl_command tcmds[] =
{
  { "CSV", "Set Comma Separated Values Mode", csv_cmd, TINYCL_PARM_INT, TINYCL_PARM_END },
  { "ATTEN", "Attenuator Setting", atten_cmd, TINYCL_PARM_INT, TINYCL_PARM_END },
  { "DEBUG", "Debug Messages", debug_cmd, TINYCL_PARM_INT, TINYCL_PARM_END },
  { "AVERAGES", "Set Averages", averages_cmd, TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END },
  { "IMACQ", "Immediate Acquisition", imacq_cmd, TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_INT },
  { "SETACQ", "Set Acquisition Parameters", setacq_cmd, TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END },
  { "DOACQ", "Do Acquisition Parameters", doacq_cmd, TINYCL_PARM_END },
  { "ACQ", "Acquisition of Ref/Thru", acq_cmd, TINYCL_PARM_END },
  { "SPARM", "Acquisition of Ref/Thru S parameters", sparm_cmd, TINYCL_PARM_END },
  { "SHUNT", "Series/Shunt 2 Port Impedance", shunt_cmd, TINYCL_PARM_INT, TINYCL_PARM_END },
  { "SHORT", "Short Calibration", shortcalib_cmd, TINYCL_PARM_END },
  { "OPEN", "Open Calibration", opencalib_cmd, TINYCL_PARM_END },
  { "LOAD", "Load Calibration", loadcalib_cmd, TINYCL_PARM_INT, TINYCL_PARM_END },
  { "THRU", "Two Port Calibration", twocalib_cmd, TINYCL_PARM_END },
  { "LISTCAL", "List Calibration States", listcal_cmd, TINYCL_PARM_END },
  { "WRITECAL", "Write Calibration State", writecal_cmd, TINYCL_PARM_INT, TINYCL_PARM_END },
  { "READCAL", "Read Calibration State", readcal_cmd, TINYCL_PARM_INT, TINYCL_PARM_END },
  { "HELP", "Display This Help", help_cmd, {TINYCL_PARM_END } },
  { "0", "Remote Cmd", remote_0_cmd, {TINYCL_PARM_END } },
  { "1", "Remote Cmd", remote_1_cmd, {TINYCL_PARM_END } },
};

int help_cmd(int args, tinycl_parameter *tp, void *v)
{
  tinycl_print_commands(sizeof(tcmds) / sizeof(tinycl_command), tcmds);
  return 1;
}

void loop(void)
{
  if (tinycl_task(sizeof(tcmds) / sizeof(tinycl_command), tcmds, NULL))
  {
    tinycl_do_echo = 1;
    Serial.print("> ");
  }
#ifdef VNA_TOUCHSCREEN
  if (touchscreen_enabled) touchscreen_task();
#endif
}
