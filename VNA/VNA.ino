/* VNA

*/

/*
   Copyright (c) 2018 Daniel Marks

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
#include "consoleio.h"

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
#define TOUCHSCREEN_DISABLE_PIN PB1

#ifdef VNA_LED
#define ACQUIRING_LED_PIN PC13
#endif

#ifdef VNA_TOUCHSCREEN
bool touchscreen_enabled;
#endif

#define PACETIMER TIMER2
#define PACETIMER_NVIC NVIC_TIMER2

Si5351 si5351(SI5351_BUS_BASE_ADDR,0);
Si5351 si5351b(SI5351_BUS_BASE_ADDR,1);

int pinMapADCCURPINin;
int pinMapADCVOLTPINin;
int pinMapADCCUR2PINin;

const vna_flash_header flash_header = { 0xDEADBEEF, 0xC001ACE5 };
vna_acquisition_state vna_state = {3000000u, 30000000u, 3000000u, 30000000u, 50u, 64, 1000, VNA_MAX_CAL_FREQS, VNA_MAX_ACQ_FREQS, 0, 0, 0, 0, VNA_NO_CALIB };
vna_calib_oneport *vna_1pt = NULL;
vna_calib_freq_parm vna_calib[VNA_MAX_CAL_FREQS];

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


volatile unsigned int number_to_sum = 256;
volatile unsigned int current_summed = 0;

volatile int sampCURI, sampVOLTI, sampCUR2I, sampPWR;
volatile int sampCURQ, sampVOLTQ, sampCUR2Q;

volatile short sampCUR, sampVOLT, sampCUR2;

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

void setup_port2_pins(void)
{
  pinMode(PA4, OUTPUT_OPEN_DRAIN);
  pinMode(PA5, OUTPUT_OPEN_DRAIN);
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
#ifdef VNA_LED
  pinMode(ACQUIRING_LED_PIN, OUTPUT);
#endif
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

volatile unsigned int lasttick = 0;
volatile unsigned int numphases = 4;
volatile unsigned int curphase = 0;

void ifClockInterrupt(void)
{
  unsigned int timerval = CPU_CYCLES;
  unsigned int diftick = ((unsigned int)(timerval - lasttick));
  lasttick = timerval;

  if (current_summed <= number_to_sum)
  {
    if (current_summed == 0)
    {
      sampCURI = 0; sampCURQ = 0;
      sampVOLTI = 0; sampVOLTQ = 0;
      sampCUR2I = 0; sampCUR2Q = 0;
      sampPWR = 0;
    }
    current_summed++;
    if (current_summed == 1) return;
    timer_pause(PACETIMER);
    timer_set_count(PACETIMER, 0);
    if (current_summed > number_to_sum)
      return;
    timerval = diftick / numphases;
    if (timerval > (F_CPU / 200000u))
    {
      timer_set_reload(PACETIMER, timerval);
      //    timer_set_compare(PACETIMER,1,0);
      timer_generate_update(PACETIMER);
      curphase = 0;
      timer_resume(PACETIMER);
    }
  }
}

void timerContInterrupt(void)
{
  if (curphase < numphases)
  {
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

  nvic_irq_set_priority(PACETIMER_NVIC, 2);
  timer_init(PACETIMER);
  timer_attach_interrupt(PACETIMER, 1, timerContInterrupt);
}

void vna_setup_remote_serial(void)
{
  console_setExternalSerial(vna_state.remote ? &Serial1 : NULL);
}

#ifdef VNA_LED
void touchscreen_led(int c)
{
  digitalWrite(ACQUIRING_LED_PIN, c ? LOW : HIGH);
}
#endif

void setup() {
  initialize_cortex_m3_cycle_counter();
  setup_pins();

  Serial1.begin(38400);
  vna_initialize_si5351();

#ifdef VNA_TOUCHSCREEN
  tcal.iscal = false;
  if (touchscreen_enabled) touchscreen_setup();
#endif

  setup_if_clock();
  setup_analog(0);
  vna_readcal(1);
  vna_setup_remote_serial();
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

int setup_frequency_acquire(unsigned int frequency)
{
  if ((frequency < VNA_MIN_FREQ) || (frequency > VNA_MAX_FREQ))
    return -1;

  rcc_clk_enable(RCC_I2C1);
  si5351.output_enable(SI5351_CLK0, 0);
#ifdef SI5351_SINGLE
  si5351.output_enable(SI5351_CLK1, 0);
#else
  setup_port2_pins();
  si5351b.output_enable(SI5351_CLK0, 0);
#endif
  if (frequency < VNA_FREQ_3X)
  {
    si5351.set_freq(frequency * SI5351_FREQ_MULT, SI5351_CLK0);
#ifdef SI5351_SINGLE
    si5351.set_freq((frequency + VNA_NOMINAL_1X_IF_FREQ) * SI5351_FREQ_MULT, SI5351_CLK1);
#else
    setup_port2_pins();
    si5351b.set_freq((frequency + VNA_NOMINAL_1X_IF_FREQ) * SI5351_FREQ_MULT, SI5351_CLK0);
#endif
    numphases = 4;
  } else if (frequency < VNA_FREQ_5X)
  {
    frequency = frequency / 3;
    si5351.set_freq(frequency * SI5351_FREQ_MULT, SI5351_CLK0);
#ifdef SI5351_SINGLE
    si5351.set_freq((frequency + VNA_NOMINAL_3X_IF_FREQ) * SI5351_FREQ_MULT, SI5351_CLK1);
#else
    setup_port2_pins();
    si5351b.set_freq((frequency + VNA_NOMINAL_3X_IF_FREQ) * SI5351_FREQ_MULT, SI5351_CLK0);
#endif
    numphases = 12;
  } else if (frequency < VNA_FREQ_7X)
  {
    frequency = frequency / 5;
    si5351.set_freq(frequency * SI5351_FREQ_MULT, SI5351_CLK0);
#ifdef SI5351_SINGLE
    si5351.set_freq((frequency + VNA_NOMINAL_5X_IF_FREQ) * SI5351_FREQ_MULT, SI5351_CLK1);
#else
    setup_port2_pins();
    si5351b.set_freq((frequency + VNA_NOMINAL_5X_IF_FREQ) * SI5351_FREQ_MULT, SI5351_CLK0);
#endif
    numphases = 20;
  } else 
  {
    frequency = frequency / 7;
    si5351.set_freq(frequency * SI5351_FREQ_MULT, SI5351_CLK0);
#ifdef SI5351_SINGLE
    si5351.set_freq((frequency + VNA_NOMINAL_7X_IF_FREQ) * SI5351_FREQ_MULT, SI5351_CLK1);
#else
   setup_port2_pins();
   si5351b.set_freq((frequency + VNA_NOMINAL_7X_IF_FREQ) * SI5351_FREQ_MULT, SI5351_CLK0);
#endif
    numphases = 28;
  }  
  delay(2);
  si5351.output_enable(SI5351_CLK0, 1);
#ifdef SI5351_SINGLE
  si5351.output_enable(SI5351_CLK1, 1);
#else
  setup_port2_pins();
  si5351b.output_enable(SI5351_CLK0, 1);
#endif
  rcc_clk_disable(RCC_I2C1);
  digitalWrite(ATTEN_PIN, vna_state.atten ? HIGH : LOW);
  return 0;
}

void vna_initialize_si5351()
{
  rcc_clk_enable(RCC_I2C1);
#ifdef SI5351_SINGLE
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 27000000u, 0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_6MA);
  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_6MA);
  si5351.set_ms_source(SI5351_CLK0, SI5351_PLLA);
  si5351.set_ms_source(SI5351_CLK1, SI5351_PLLB);
#else
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 27000000u, 0);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_6MA);
  si5351.drive_strength(SI5351_CLK2, SI5351_DRIVE_6MA);
  si5351.set_ms_source(SI5351_CLK0, SI5351_PLLA);
  si5351.set_ms_source(SI5351_CLK1, SI5351_PLLB);
  si5351.set_ms_source(SI5351_CLK2, SI5351_PLLB);
  si5351.set_freq(27000000ull * SI5351_FREQ_MULT, SI5351_CLK2);
  setup_port2_pins();
  si5351b.init(SI5351_CRYSTAL_LOAD_8PF, 27000000u, 0);
  si5351b.drive_strength(SI5351_CLK0, SI5351_DRIVE_6MA);
  si5351b.set_ms_source(SI5351_CLK0, SI5351_PLLA);
#endif
  rcc_clk_disable(RCC_I2C1);
}

vna_acquire_current_state vna_operation_acquire_dataset(vna_acquire_state *vas)
{
  vna_acquisition_state *vs = vas->vs;
  unsigned int freq = vs->startfreq + vas->freqstep * vas->n;

  if (vas->vacs == VNA_ACQUIRE_RECORDED_STATE)
  {
    number_to_sum = 0;
    if (vas->wait_for_end)
    {
      setup_frequency_acquire(freq);
      timer_pause(PACETIMER);
      timer_attach_interrupt(PACETIMER, 1, timerContInterrupt);
      timer_set_prescaler(PACETIMER, 0);
      timer_set_mode(PACETIMER, 1, TIMER_OUTPUT_COMPARE);
      timer_set_count(PACETIMER, 0);
      timer_set_reload(PACETIMER, 65535);
      timer_set_compare(PACETIMER, 1, 0);
      timer_generate_update(PACETIMER);
    }
    vas->vacs = VNA_ACQUIRE_RECORDED_STATE_WAITING;
    vas->inittime = CPU_CYCLES;
    current_summed = 0;
    number_to_sum = vas->wait_for_end ? 12 : vs->num_averages;
  }
  if (vas->vacs != VNA_ACQUIRE_RECORDED_STATE_WAITING)
    return vas->vacs;
#ifdef VNA_LED
  touchscreen_led(CPU_CYCLES & 0x400000);
#endif
  if (current_summed <= number_to_sum)
  {
    unsigned int curtime = CPU_CYCLES;
    if (((unsigned int)(curtime - vas->inittime)) > vas->timeout)
      vas->vacs = VNA_ACQUIRE_TIMEOUT;
    return vas->vacs;
  }
  if (vas->wait_for_end)
  {
    vas->wait_for_end = false;
    vas->vacs = VNA_ACQUIRE_RECORDED_STATE;
    return vas->vacs;
  }
  if (vas->vado != NULL)
  {
    vna_acquire_dataset_state vads;
    vads.n = vas->n;
    vads.total = vs->nfreqs;
    vads.freq = freq;
    vads.volti = sampVOLTI;
    vads.voltq = sampVOLTQ;
    vads.curi = sampCURI;
    vads.curq = sampCURQ;
    vads.cur2i = sampCUR2I;
    vads.cur2q = sampCUR2Q;
    vas->vado(&vads, vas->vado_v);
  }
  vas->n++;
  if (vas->n >= vs->nfreqs)
  {
    vas->vacs = VNA_ACQUIRE_COMPLETE;
    return vas->vacs;
  }
  if (vas->check_interruption)
  {
    if (console_inchar() == '!')
    {
      vas->vacs = VNA_ACQUIRE_ABORT;
      return vas->vacs;
    }
#ifdef VNA_TOUCHSCREEN
    if (touchscreen_enabled && touchscreen_abort())
    {
      vas->vacs = VNA_ACQUIRE_ABORT;
      return vas->vacs;
    }
#endif
  }
  vas->wait_for_end = true;
  vas->vacs = VNA_ACQUIRE_RECORDED_STATE;
  return vas->vacs;
}

vna_acquire_current_state vna_setup_acquire_dataset(vna_acquire_state *vas, vna_acquisition_state *vs, vna_acquire_dataset_operation vado, void *v)
{
  number_to_sum = 0;
  vas->vs = vs;
  vas->wait_for_end = true;
  vas->n = 0;
  vas->vado = vado;
  vas->vado_v = v;
  vas->timeout = vs->timeout * (F_CPU / 1000u);
  vas->freqstep = (vs->endfreq - vs->startfreq) / vs->nfreqs;
  vas->vacs = VNA_ACQUIRE_RECORDED_STATE;
  vas->check_interruption = true;
  vna_initialize_si5351();
  return vas->vacs;
}

bool vna_acquire_dataset(vna_acquisition_state *vs, vna_acquire_dataset_operation vado, void *v)
{
  vna_acquire_state vas;

  vna_setup_acquire_dataset(&vas, vs, vado, v);
  do
  {
    vna_operation_acquire_dataset(&vas);
  } while ((vas.vacs != VNA_ACQUIRE_TIMEOUT) && (vas.vacs != VNA_ACQUIRE_ABORT) && (vas.vacs != VNA_ACQUIRE_COMPLETE));
  return (vas.vacs == VNA_ACQUIRE_COMPLETE);
}

int vna_set_averages(unsigned short averages, unsigned short timeout)
{
  if ((averages >= 1) && (averages <= 50000))
    vna_state.num_averages = averages;
  if ((timeout >= 1) && (timeout <= 50000))
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
  vna_set_averages(tp[0].ti.i, tp[1].ti.i);
  console_print("Averages=");
  console_println(vna_state.num_averages);
  console_print("Timeout=");
  console_println(vna_state.timeout);
  return 1;
}

int csv_cmd(int args, tinycl_parameter* tp, void *v)
{
  int n;
  vna_acquisition_state *vs = &vna_state;
  vs->csv = tp[0].ti.i;
  console_print("CSV=");
  console_println(vs->csv);
  return 1;
}

int debug_cmd(int args, tinycl_parameter* tp, void *v)
{
  int n;
  setDebugMsgMode(tp[0].ti.i);
  console_print("DEBUG=");
  console_println(debugmsg_state);
  return 1;
}

int remote_serial_cmd(int args, tinycl_parameter* tp, void *v)
{
  int n;
  vna_acquisition_state *vs = &vna_state;
  vs->remote = tp[0].ti.i;
  console_print("remote=");
  console_println(vs->remote);
  vna_setup_remote_serial();
  return 1;
}

const char *atten_strings[3] = { "0 Off", "1 On" };

int atten_cmd(int args, tinycl_parameter* tp, void *v)
{
  int n = tp[0].ti.i;

  if ((n < 0) || (n > 1))
  {
    console_println("Invalid setting");
    return 1;
  }
  vna_state.atten = n;
  console_print("atten=");
  console_println(atten_strings[vna_state.atten]);
  return 1;
}

const char *series_shunt_strings[3] = { "0 Series", "1 Shunt" };

int shunt_cmd(int args, tinycl_parameter* tp, void *v)
{
  int n = tp[0].ti.i;

  if ((n < 0) || (n > 1))
  {
    console_println("Invalid setting");
    return 1;
  }
  vna_state.series_shunt_two = n;
  console_print("impedance-mode=");
  console_println(series_shunt_strings[vna_state.series_shunt_two]);
  return 1;
}


int vna_display_dataset_operation(vna_acquire_dataset_state *vads, void *va)
{
  if (vna_state.csv)
  {
    console_print(vads->freq);
    console_print(",");
    console_print(vads->volti);
    console_print(",");
    console_print(vads->voltq);
    console_print(",");
    console_print(vads->curi);
    console_print(",");
    console_print(vads->curq);
    console_print(",");
    console_print(vads->cur2i);
    console_print(",");
    console_println(vads->cur2q);
  } else
  {
    console_print("Freq ");
    console_print(vads->freq);
    console_print(": Volt (");
    console_print(vads->volti);
    console_print(",");
    console_print(vads->voltq);
    console_print(") Cur1 (");
    console_print(vads->curi);
    console_print(",");
    console_print(vads->curq);
    console_print(") Cur2 (");
    console_print(vads->cur2i);
    console_print(",");
    console_print(vads->cur2q);
    console_println(")");
  }
}

int doacq_cmd(int args, tinycl_parameter* tp, void *v)
{
  int n;
  vna_acquisition_state *vs = &vna_state;
  vna_acquire_dataset(vs, vna_display_dataset_operation, NULL);
  return 1;
}

int vna_calset_frequencies(unsigned int nfreqs, unsigned int startfreq, unsigned int endfreq)
{
  startfreq = ((startfreq + 500u) / 1000u) * 1000u;
  endfreq = ((endfreq + 500u) / 1000u) * 1000u;
  if ((startfreq >= VNA_MIN_FREQ) && (endfreq > startfreq) && (endfreq <= VNA_MAX_FREQ))
  {
    vna_state.cal_startfreq = startfreq;
    vna_state.cal_endfreq = endfreq;
  } else return 0;
  if ((nfreqs >= 1) && (nfreqs <= VNA_MAX_CAL_FREQS)) vna_state.cal_nfreqs = nfreqs;
  else return 0;
  vna_state.startfreq = vna_state.cal_startfreq;
  vna_state.endfreq = vna_state.cal_endfreq;
  vna_state.nfreqs = vna_state.cal_nfreqs;
  reset_calib_state();
  return 1;
}

int vna_set_frequencies(unsigned int nfreqs, unsigned int startfreq, unsigned int endfreq)
{
  if ((startfreq >= vna_state.cal_startfreq) && (endfreq > startfreq) && (endfreq <= vna_state.cal_endfreq))
  {
    vna_state.startfreq = startfreq;
    vna_state.endfreq = endfreq;
  } else return 0;
  if ((nfreqs >= 1) && (nfreqs <= VNA_MAX_ACQ_FREQS)) vna_state.nfreqs = nfreqs;
  else return 0;
  return 1;
}

int setcal_cmd(int args, tinycl_parameter* tp, void *v)
{
  unsigned int nfreqs = tp[0].ti.i;
  unsigned int startfreq = tp[1].ti.i;
  unsigned int endfreq = tp[2].ti.i;
  bool statereset = (vna_state.calib_state != VNA_NO_CALIB);
  if (!vna_calset_frequencies(nfreqs, startfreq, endfreq))
  {
    console_println("Invalid Parameters");
    return 1;
  }
  console_print("Cal Start Frequency = ");
  console_println(vna_state.startfreq);
  console_print("Cal End Frequency = ");
  console_println(vna_state.endfreq);
  console_print("Cal Number Frequencies = ");
  console_println(vna_state.nfreqs);
  if (statereset)
    console_println("Calibration state reset");
  return 1;
}

int setacq_cmd(int args, tinycl_parameter* tp, void *v)
{
  unsigned int nfreqs = tp[0].ti.i;
  unsigned int startfreq = tp[1].ti.i;
  unsigned int endfreq = tp[2].ti.i;
  if (!vna_set_frequencies(nfreqs, startfreq, endfreq))
  {
    console_println("Invalid Parameters");
    return 1;
  }
  console_print("Start Frequency = ");
  console_println(vna_state.startfreq);
  console_print("End Frequency = ");
  console_println(vna_state.endfreq);
  console_print("Number Frequencies = ");
  console_println(vna_state.nfreqs);
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
    console_println("Invalid number of frequencies");
    return 1;
  }
  if ((vs.startfreq < VNA_MIN_FREQ) || (vs.endfreq < vs.startfreq) && (vs.endfreq > VNA_MAX_FREQ))
  {
    console_println("Invalid maximum or minimum frequency");
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
    mini_snprintf(s, sizeof(s) - 1, "Open freq %u zo=%06f + j%06f", vads->freq, float2int32(v1pt[vads->n].zo.real), float2int32(v1pt[vads->n].zo.imag));
    console_println(s);
  }
}

int vna_short_dataset_operation(vna_acquire_dataset_state *vads, void *va)
{
  vna_calib_oneport *v1pt = (vna_calib_oneport *)va;
  v1pt[vads->n].zs = Complex((float)vads->volti, (float)vads->voltq) / Complex((float)vads->curi, (float)vads->curq);
  if (debugmsg_state)
  {
    char s[80];
    mini_snprintf(s, sizeof(s) - 1, "Short freq %u zs=%06f + j%06f", vads->freq, float2int32(v1pt[vads->n].zs.real), float2int32(v1pt[vads->n].zs.imag));
    console_println(s);
  }
}

int vna_load_dataset_operation(vna_acquire_dataset_state *vads, void *va)
{
  vna_calib_oneport *v1pt = (vna_calib_oneport *)va;
  v1pt[vads->n].zt = Complex((float)vads->volti, (float)vads->voltq) / Complex((float)vads->curi, (float)vads->curq);
  vna_calib[vads->n].s2 = Complex((float)vads->cur2i, (float)vads->cur2q) / ((float) vna_state.num_averages);
  if (debugmsg_state)
  {
    char s[80];
    mini_snprintf(s, sizeof(s) - 1, "Load freq %u zt=%06f + j%06f", vads->freq, float2int32(v1pt[vads->n].zt.real), float2int32(v1pt[vads->n].zt.imag));
    console_println(s);
  }
}

int vna_allocate_1pt_calib(void)
{
  if (vna_1pt != NULL) return 1;
  if ((vna_1pt = (vna_calib_oneport *) malloc(sizeof(vna_calib_oneport) * vna_state.cal_nfreqs)) == NULL)
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
  for (n = 0; n < vna_state.cal_nfreqs; n++)
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

void vna_set_calib_acquisition_state(vna_acquisition_state *vs)
{
  *vs = vna_state;
  vs->nfreqs = vna_state.cal_nfreqs;
  vs->startfreq = vna_state.cal_startfreq;
  vs->endfreq = vna_state.cal_endfreq;
}

int vna_opencalib(void)
{
  vna_acquisition_state vs;
  vna_set_calib_acquisition_state(&vs);
  if (!vna_allocate_1pt_calib()) return 0;
  if (!vna_acquire_dataset(&vs, vna_open_dataset_operation, (void *)vna_1pt)) return 0;
  vna_state.calib_state |= VNA_OPEN_CALIB;
  return vna_complete_1pt_calib() ? 2 : 1;
}

int opencalib_cmd(int args, tinycl_parameter* tp, void *v)
{
  console_println("Open calibration");
  switch (vna_opencalib())
  {
    case 0: console_println("Calibration aborted");
      return 1;
      break;
    case 1: console_println("\r\nOpen calibration successful");
      break;
    case 2: console_println("\r\n1 Port calibration successful");
      break;
  }
  return 1;
}

int vna_shortcalib(void)
{
  vna_acquisition_state vs;
  vna_set_calib_acquisition_state(&vs);
  if (!vna_allocate_1pt_calib()) return 0;
  if (!vna_acquire_dataset(&vs, vna_short_dataset_operation, (void *)vna_1pt)) return 0;
  vna_state.calib_state |= VNA_SHORT_CALIB;
  return vna_complete_1pt_calib() ? 2 : 1;
}

int shortcalib_cmd(int args, tinycl_parameter* tp, void *v)
{
  console_println("Short calibration");
  switch (vna_shortcalib())
  {
    case 0: console_println("Calibration aborted");
      return 1;
      break;
    case 1: console_println("\r\nShort calibration successful");
      break;
    case 2: console_println("\r\n1 Port calibration successful");
      break;
  }
  return 1;
}

int vna_loadcalib(void)
{
  vna_acquisition_state vs;
  vna_set_calib_acquisition_state(&vs);
  if (!vna_allocate_1pt_calib()) return 0;
  if (!vna_acquire_dataset(&vs, vna_load_dataset_operation, (void *)vna_1pt)) return 0;
  vna_state.calib_state |= VNA_LOAD_CALIB;
  return vna_complete_1pt_calib() ? 2 : 1;
}

int loadcalib_cmd(int args, tinycl_parameter* tp, void *v)
{
  console_println("Load calibration");
  vna_set_characteristic_impedance(tp[0].ti.i);
  switch (vna_loadcalib())
  {
    case 0: console_println("Calibration aborted");
      return 1;
      break;
    case 1: console_println("\r\nLoad calibration successful");
      break;
    case 2: console_println("\r\n1 Port calibration successful");
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
  cur2pt = cur2pt - vc->s2 * ((float)vna_state.num_averages);
  vc->z2 = vn / cur2pt;
  vc->i2 = in / cur2pt;
  if (debugmsg_state)
  {
    char s[80];
    mini_snprintf(s, sizeof(s) - 1, "Thru freq %u z2=%06f + j%06f  i2 = %06f + j%06f", vads->freq, float2int32(vc->z2.real), float2int32(vc->z2.imag), float2int32(vc->i2.real), float2int32(vc->i2.imag));
    console_println(s);
  }
}

int vna_thrucal(void)
{
  vna_acquisition_state vs;
  vna_set_calib_acquisition_state(&vs);
  if (!(vna_state.calib_state & VNA_VALID_CALIB_1PT)) return 0;
  if (!vna_acquire_dataset(&vs, vna_twocalib_dataset_operation, NULL))
  {
    vna_state.calib_state &= ~VNA_VALID_CALIB_2PT;
    return 1;
  }
  vna_state.calib_state |= VNA_VALID_CALIB_2PT;
  return 2;
}

int twocalib_cmd(int args, tinycl_parameter* tp, void *v)
{
  console_println("Thru calibration");
  switch (vna_thrucal())
  {
    case 0: console_println("Two Port Must Have Valid One Port Cal");
      break;
    case 1: console_println("Calibration aborted");
      break;
    case 2:  console_println("\r\nTwo port calibration successful");
      break;
  }
  return 1;
}

int vna_rtr_impedance_display(int n, int total, unsigned int freq, bool ch2, Complex imp, Complex zthru)
{
  char s[80];
  mini_snprintf(s, sizeof(s) - 1, vna_state.csv ? "%u,%04f,%04f" : "Freq: %u Z (%04f,%04f)", freq, float2int32(imp.real), float2int32(imp.imag));
  console_print(s);
  if (ch2)
  {
    mini_snprintf(s, sizeof(s) - 1, vna_state.csv ? ",%04f,%04f" : " ZT (%04f,%04f)", float2int32(zthru.real), float2int32(zthru.imag));
    console_print(s);
  }
  console_println("");
}

void vna_get_interpolated_cal_parameters(unsigned int freq, vna_calib_freq_parm &vcfp, bool twoport)
{
  if (freq < vna_state.cal_startfreq)
  {
    vcfp = vna_calib[0];
    return;
  }
  if (freq >= vna_state.cal_endfreq)
  {
    vcfp = vna_calib[vna_state.cal_nfreqs - 1];
    return;
  }
  uint32_t freqspan = vna_state.cal_endfreq - vna_state.cal_startfreq;
  uint64_t prod = ((uint64_t)(freq - vna_state.cal_startfreq)) * ((uint64_t)vna_state.cal_nfreqs);
  uint32_t intindex = prod / ((uint64_t)freqspan);
  float fracindex = ((float)(prod % ((uint64_t)freqspan))) / ((float)freqspan);
  uint32_t intindex1 = (intindex == (vna_state.cal_nfreqs - 1)) ? (vna_state.cal_nfreqs - 1) : intindex + 1;
  float fracindex1 = 1.0f - fracindex;
  vcfp.b = vna_calib[intindex].b * fracindex1 + vna_calib[intindex1].b * fracindex;
  vcfp.c = vna_calib[intindex].c * fracindex1 + vna_calib[intindex1].c * fracindex;
  vcfp.d = vna_calib[intindex].d * fracindex1 + vna_calib[intindex1].d * fracindex;
  if (twoport)
  {
    vcfp.i2 = vna_calib[intindex].i2 * fracindex1 + vna_calib[intindex1].i2 * fracindex;
    vcfp.s2 = vna_calib[intindex].s2 * fracindex1 + vna_calib[intindex1].s2 * fracindex;
    vcfp.z2 = vna_calib[intindex].z2 * fracindex1 + vna_calib[intindex1].z2 * fracindex;
  }
  return;
}

int vna_display_acq_operation(vna_acquire_dataset_state *vads, void *va)
{
  Complex voltpt((float)vads->volti, (float)vads->voltq);
  Complex curpt((float)vads->curi, (float)vads->curq);
  vna_calib_freq_parm vc;
  vna_get_interpolated_cal_parameters(vads->freq, vc, (vna_state.calib_state & VNA_VALID_CALIB_2PT));
  Complex vn = voltpt + curpt * vc.b;
  Complex in = curpt * vc.d + voltpt * vc.c;
  Complex imp = vn / in;
  Complex zthru;

  if (vna_state.calib_state & VNA_VALID_CALIB_2PT)
  {
    Complex curpt2((float)vads->cur2i, (float)vads->cur2q);
    curpt2 = curpt2 - vc.s2 * ((float)vna_state.num_averages);
    if (vna_state.series_shunt_two)
    {
      zthru = (curpt2 * vc.z2) / (in - curpt2 * vc.i2);
    } else
    {
      zthru = (vn - curpt2 * vc.z2) / (curpt2 * vc.i2);
    }
  }
  ((vna_report_trans_reflected)va)(vads->n, vads->total, vads->freq, (vna_state.calib_state & VNA_VALID_CALIB_2PT) != 0, imp.conj(), zthru.conj());
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
    case 0: console_println("Can only be performed after 1 or 2 port calibration");
      break;
    case 1: console_println("Acquisition aborted");
      break;
    case 2: console_println("-----");
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
  mini_snprintf(s, sizeof(s) - 1, vna_state.csv ? "%u,%04f,%04f" : "Freq %u: S11 (%04f,%04f)", freq, float2int32(s11db), float2int32(s11deg));
  console_print(s);
  if (ch2)
  {
    mini_snprintf(s, sizeof(s) - 1, vna_state.csv ? ",%04f,%04f" : " S21 (%04f,%04f)", float2int32(s21db), float2int32(s21deg));
    console_print(s);
  }
  console_println("");
}

int vna_display_sparm_operation(vna_acquire_dataset_state *vads, void *va)
{
  Complex voltpt((float)vads->volti, (float)vads->voltq);
  Complex curpt((float)vads->curi, (float)vads->curq);
  vna_calib_freq_parm vc;
  vna_get_interpolated_cal_parameters(vads->freq, vc, (vna_state.calib_state & VNA_VALID_CALIB_2PT));
  Complex vn = voltpt + curpt * vc.b;
  Complex in = curpt * vc.d + voltpt * vc.c;
  Complex imp = vn / in;
  float z0 = (float)vna_state.char_impedance;
  Complex s11 = (imp - z0) / (imp + z0);
  Complex s21;
  if (vna_state.calib_state & VNA_VALID_CALIB_2PT)
  {
    Complex curpt2((float)vads->cur2i, (float)vads->cur2q);
    curpt2 = curpt2 - vc.s2 * ((float)vna_state.num_averages);
    s21 = curpt2 * (vc.z2 + vc.i2 * z0) / (vn + in * z0);
  }
  ((vna_report_trans_reflected)va)(vads->n, vads->total, vads->freq, (vna_state.calib_state & VNA_VALID_CALIB_2PT) != 0, s11.conj(), s21.conj());
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
    case 0: console_println("Can only be performed after 1 or 2 port calibration");
      break;
    case 1: console_println("Acquisition aborted");
      break;
    case 2: console_println("-----");
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
    console_println("There is no calibration to write");
    return -1;
  }
  if ((n < 1) || (n > NUM_FLASH_PAGES))
  {
    console_println("Invalid calibration number");
    return 1;
  }
  console_print("Writing calibration ");
  console_print(n);
  console_println(vna_writecal(n) ? ": written" : ": failed");
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
    console_println("Invalid calibration number");
    return 1;
  }
  if (vna_readcal(n))
  {
    console_print("Read calibration ");
    console_println(n);
  } else
    console_println("No calibration to retrieve");
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
  if (readflashstruct((void *)flash_pages[n - 1], 2, vp, b))
  {
    if ((rd_flash_header.flash_header_1 == flash_header.flash_header_1) && (rd_flash_header.flash_header_2 == flash_header.flash_header_2))
    {
      mini_snprintf(s, len, "%d: Start %u End %u%c# %u Impedance %u Avgs %u %s",
                    n,
                    rd_vna_state.cal_startfreq,
                    rd_vna_state.cal_endfreq,
                    splitchar,
                    rd_vna_state.cal_nfreqs,
                    rd_vna_state.char_impedance,
                    rd_vna_state.num_averages,
                    (rd_vna_state.calib_state & VNA_VALID_CALIB_2PT) ? "2 Port" :
                    ((rd_vna_state.calib_state & VNA_VALID_CALIB_1PT) ? "1 Port" : "No Cal"));
      return 1;
    } else
    {
      mini_snprintf(s, len, "%d: No Calibration", n);
    }
  }
  return 0;
}

int listcal_cmd(int args, tinycl_parameter* tp, void *v)
{
  int n;
  char s[120];
  console_println("Calibrations:");
  for (n = 1; n <= NUM_FLASH_PAGES; n++)
  {
    vna_calentry(n, s, sizeof(s) - 1);
    console_println(s);
  }
  console_println("-----");
  return 1;
}

unsigned int get_number_serial()
{
  unsigned int num = 0;

  for (;;)
  {
    int c;
    while ((c = console_inchar()) == -1);
    if ((c >= '0') && (c <= '9')) num = num * 10 + c - '0';
    if (c == '\r') break;
  }
  return num;
}

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
  short dbint = 344.0f - db * 17.0f;
  short degint = (deg < 0.0f ? 360.0f + deg : deg) * (1024.0f / 180.0f);
  dbint = (dbint < 0) ? 0 : dbint;
  degint = (degint < 5) ? 5 : degint;
  degint = (degint > 2042) ? 2042 : degint;
  b[0] = degint & 0xFF;
  b[1] = degint >> 8;
  b[2] = dbint & 0xFF;
  b[3] = dbint >> 8;
  console_printchar((char)b[0]);
  console_printchar((char)b[1]);
  console_printchar((char)b[2]);
  console_printchar((char)b[3]);
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
  uint32_t start_frequency = (((uint64_t)frequency_tuning_word) * VNA_FREQUENCY_TUNING_CONVERSION_NUMERATOR) / VNA_FREQUENCY_TUNING_CONVERSION_DENOMINATOR;
  uint32_t step_frequency = (((uint64_t)frequency_tuning_step) * VNA_FREQUENCY_TUNING_CONVERSION_NUMERATOR) / VNA_FREQUENCY_TUNING_CONVERSION_DENOMINATOR;
  uint32_t stop_frequency = start_frequency + number_of_samples * step_frequency;
  if ((start_frequency != vna_state.startfreq) || (stop_frequency != vna_state.endfreq) || (number_of_samples != vna_state.nfreqs))
    vna_set_frequencies(number_of_samples, start_frequency, stop_frequency);
  vna_acquire_sparm(vna_rtr_remote_cmd);
  for (i = vna_rtr_remote_cmd_samples; i < number_of_samples; i++)
  {
    console_printchar((char)0xFF);
    console_printchar((char)0x03);
    console_printchar((char)0xFF);
    console_printchar((char)0x03);
  }
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

#define VNA_NOISE_MEASURE
#ifdef VNA_NOISE_MEASURE
int noise_cmd(int args, tinycl_parameter* tp, void *v)
{
  int i=0;
  float totalVOLT = 0.0;
  float totalCUR = 0.0;
  float totalCUR2 = 0.0;
  float total2VOLT = 0.0;
  float total2CUR = 0.0;
  float total2CUR2 = 0.0;
  
  int samples = tp[0].ti.i;
  for (i=0;i<samples;i++)
  {
    analogReadPins();
    totalVOLT += sampVOLT;
    totalCUR += sampCUR;
    totalCUR2 += sampCUR2;
    total2VOLT += ((unsigned int)sampVOLT)*((unsigned int)sampVOLT);
    total2CUR += ((unsigned int)sampCUR)*((unsigned int)sampCUR);
    total2CUR2 += ((unsigned int)sampCUR2)*((unsigned int)sampCUR2);
  }
  totalVOLT /= samples;
  totalCUR /= samples; 
  totalCUR2 /= samples;
  total2VOLT /= samples;
  total2CUR /= samples;
  total2CUR2 /= samples;
  total2VOLT = (total2VOLT-totalVOLT*totalVOLT);
  total2CUR = (total2CUR-totalCUR*totalCUR);
  total2CUR2 = (total2CUR2-totalCUR2*totalCUR2);

  char s[80];
  mini_snprintf(s,sizeof(s)-1,"Averages %03f %03f %03f",float2int32(totalVOLT),float2int32(totalCUR),float2int32(totalCUR2));
  console_println(s);
  mini_snprintf(s,sizeof(s)-1,"VAR %03f %03f %03f",float2int32(total2VOLT),float2int32(total2CUR),float2int32(total2CUR2));
  console_println(s);
  total2VOLT = sqrtf(total2VOLT);
  total2CUR = sqrtf(total2CUR);
  total2CUR2 = sqrtf(total2CUR2);
  mini_snprintf(s,sizeof(s)-1,"STD %03f %03f %03f",float2int32(total2VOLT),float2int32(total2CUR),float2int32(total2CUR2));
  console_println(s);
  return 1;  
}
#endif

#if 0
#include "JogWheel.h"
JogWheel jogwheel;
bool jogwheel_init = false;

int jogwheel_cmd(int args, tinycl_parameter* tp, void *v)
{
  char s[80];
  if (!jogwheel_init)
  {
    jogwheel_init = true;
    jogwheel.setup();
  }
  mini_snprintf(s, sizeof(s) - 1, "Jog count %d total %d int %d pushed %d", jogwheel.readCounts(), jogwheel.totalCounts(), jogwheel.readInterrupts(), jogwheel.getSelect() ? 1 : 0);
  console_println(s);
  return 1;
}
#endif

const tinycl_command tcmds[] =
{
  //  { "J", "Jogwheel=", jogwheel_cmd, TINYCL_PARM_END },
  { "CSV", "Set Comma Separated Values Mode", csv_cmd, TINYCL_PARM_INT, TINYCL_PARM_END },
#ifdef VNA_NOISE_MEASURE
  { "NOISE", "Noise", noise_cmd, TINYCL_PARM_INT, TINYCL_PARM_END },
#endif
  { "ATTEN", "Attenuator Setting", atten_cmd, TINYCL_PARM_INT, TINYCL_PARM_END },
  { "DEBUG", "Debug Messages", debug_cmd, TINYCL_PARM_INT, TINYCL_PARM_END },
  { "REMOTE", "Remote Serial", remote_serial_cmd, TINYCL_PARM_INT, TINYCL_PARM_END },
  { "AVERAGES", "Set Averages", averages_cmd, TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END },
  { "IMACQ", "Immediate Acquisition", imacq_cmd, TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_INT },
  { "SETCAL", "Set Calibration Acquisition Parameters", setcal_cmd, TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END },
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
#ifdef VNA_LED
  touchscreen_led(0);
#endif
  if (tinycl_task(sizeof(tcmds) / sizeof(tinycl_command), tcmds, NULL))
  {
    tinycl_do_echo = 1;
    console_print("> ");
  }
#ifdef VNA_TOUCHSCREEN
  if (touchscreen_enabled) touchscreen_task();
#endif
}
