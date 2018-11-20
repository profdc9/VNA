/* VNA
 *
 */

#include <Wire.h>
#include "si5351.h"
#include "tinycl.h"
#include "debugmsg.h"

#define IFCLOCK_PIN PB15
#define IFCLOCK_PIN_CTR PA15
#define IFCLOCK_PORT2_PIN PA8
#define AD_CUR_PIN PA0
#define AD_VOLT_PIN PB0
#define AD_CUR2_PIN PA2

Si5351 si5351;

int pinMapADCCURPINin;
int pinMapADCVOLTPINin;
int pinMapADCCUR2PINin;

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

volatile int sampCURI, sampVOLTI, sampCUR2I;
volatile int sampCURQ, sampVOLTQ, sampCUR2Q;

unsigned short analogReadPins(void)
{
  ADC1->regs->SQR3 = pinMapADCCURPINin;
  ADC1->regs->CR2 |= ADC_CR2_SWSTART; 
  ADC2->regs->SQR3 = pinMapADCVOLTPINin;
  ADC2->regs->CR2 |= ADC_CR2_SWSTART; 

  while (!(ADC1->regs->SR & ADC_SR_EOC));
  sampCUR=(ADC1->regs->DR & ADC_DR_DATA);
  
  ADC1->regs->SQR3 = pinMapADCCUR2PINin;
  ADC1->regs->CR2 |= ADC_CR2_SWSTART; 
  
  while (!(ADC2->regs->SR & ADC_SR_EOC));
  sampVOLT=(ADC2->regs->DR & ADC_DR_DATA);

  while (!(ADC1->regs->SR & ADC_SR_EOC));
  sampCUR2=(ADC1->regs->DR & ADC_DR_DATA);
}

void setup_pins(void)
{
  pinMode(IFCLOCK_PIN,INPUT);
  pinMode(IFCLOCK_PIN_CTR,INPUT);
  pinMode(IFCLOCK_PORT2_PIN,INPUT);
  pinMode(AD_CUR_PIN,INPUT_ANALOG);
  pinMode(AD_VOLT_PIN,INPUT_ANALOG);
  pinMode(AD_CUR2_PIN,INPUT_ANALOG);
}

#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT
#define DEMCR_TRCENA    0x01000000

HardwareTimer timeb(3);

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

unsigned int timephase[30];

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
    }
    current_summed++;
    timephase[0] = timerval;
    timer_pause(TIMER3);
    timer_set_count(TIMER3,0);
    timer_set_reload(TIMER3,diftick/numphases);
//    timer_set_compare(TIMER3,1,0);
    timer_generate_update(TIMER3);  
    curphase = 0;
    timer_resume(TIMER3);
  } 
}

void timerContInterrupt(void)
{
  unsigned int timerval = CPU_CYCLES;

  if (curphase < numphases)
  {
    timephase[curphase] = timerval;
    analogReadPins();
    switch (curphase & 0x03)
    {
       case 0: sampCURI += sampCUR;
               sampVOLTI += sampVOLT;
               sampCUR2I += sampCUR2;
               break; 
       case 1: sampCURQ += sampCUR;
               sampVOLTQ += sampVOLT;
               sampCUR2Q += sampCUR2;
               break; 
       case 2: sampCURI -= sampCUR;
               sampVOLTI -= sampVOLT;
               sampCUR2I -= sampCUR2;
               break; 
       case 3: sampCURQ -= sampCUR;
               sampVOLTQ -= sampVOLT;
               sampCUR2Q -= sampCUR2;
               break; 
    }
    curphase++; 
  } 
}

void setup_if_clock(void)
{
  nvic_irq_set_priority(NVIC_EXTI_15_10, 2);
  attachInterrupt(IFCLOCK_PIN,ifClockInterrupt,RISING);

  nvic_irq_set_priority(NVIC_TIMER3, 2);
  timeb.pause();
  timeb.setCount(0);
  timeb.setPrescaleFactor(1);
  timeb.setOverflow(65535);
  timeb.setCompare(TIMER_CH1, 0);
  timeb.attachInterrupt(1,timerContInterrupt);
  timeb.refresh();
  //timeb.resume();
}


void setup() {
    initialize_cortex_m3_cycle_counter();
    setup_pins();
    setup_if_clock();
    setup_analog(0);
  
  // put your setup code here, to run once:
     si5351.init(SI5351_CRYSTAL_LOAD_8PF, 27000000u, 0
     );
     si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_6MA);
     si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_6MA);
    si5351.set_freq(10000000ull * SI5351_FREQ_MULT, SI5351_CLK0);
    si5351.set_freq(10010000ull * SI5351_FREQ_MULT, SI5351_CLK1);
    si5351.output_enable(SI5351_CLK0, 1);
    si5351.output_enable(SI5351_CLK1, 1);
}

int print_cmd(int args, tinycl_parameter *tp, void *v)
{
  Serial.print("printing: ");
  Serial.println(tp[0].ts.str);
  return 0;
}

int print2_cmd(int args, tinycl_parameter *tp, void *v)
{
  Serial.print("printing 1: ");
  Serial.println(tp[0].ts.str);
  Serial.print("printing 2: ");
  Serial.println(tp[1].ts.str);
  return 0;
}

int add_cmd(int args, tinycl_parameter *tp, void *v)
{
  Serial.print("adding ");
  Serial.print(tp[0].ti.i);
  Serial.print("+");
  Serial.print(tp[1].ti.i);
  Serial.print("=");
  Serial.println(tp[0].ti.i+tp[1].ti.i);
  return 0;
}

#define VNA_MAX_FREQS 100
#define VNA_MIN_FREQ 1000000u
#define VNA_MAX_FREQ 470000000u
#define VNA_FREQ_3X 180000000u
#define VNA_NOMINAL_1X_IF_FREQ 10000u
#define VNA_NOMINAL_3X_IF_FREQ 3333u

unsigned int vna_mode = 1; 
unsigned int vna_nfreqs = 100;
unsigned int vna_startfreq = 3000000u;
unsigned int vna_endfreq = 30000000u;
unsigned int vna_num_averages = 64;

typedef struct _vna_data_set
{ 
  unsigned int freq[VNA_MAX_FREQS];
  int vna_p1_volt_i[VNA_MAX_FREQS];
  int vna_p1_volt_q[VNA_MAX_FREQS];
  int vna_p1_cur_i[VNA_MAX_FREQS];
  int vna_p1_cur_q[VNA_MAX_FREQS];
  int vna_p2_cur_i[VNA_MAX_FREQS];
  int vna_p2_cur_q[VNA_MAX_FREQS];
} vna_data_set;

vna_data_set vna_data;

int acquire_sample(unsigned int averages, unsigned int timeout)
{
   bool timedout = false;
   current_summed = 0;
   unsigned int inittime, curtime;

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
   if (timedout)
   {
      DEBUGMSG("ACQ: Timeout waiting for last acquisition");
      return -1;
   }
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
   if (timedout)
   {
      DEBUGMSG("ACQ: Timeout waiting for acquisition");
      return -1;
   }   
   return 0;
}

int setup_frequency_acquire(unsigned int frequency)
{
    if ((frequency < VNA_MIN_FREQ) || (frequency > VNA_MAX_FREQ))
        return -1;
    if (frequency < VNA_FREQ_3X)
    {
      si5351.set_freq(frequency * SI5351_FREQ_MULT, SI5351_CLK0);
      si5351.set_freq((frequency+VNA_NOMINAL_1X_IF_FREQ) * SI5351_FREQ_MULT, SI5351_CLK1);
      numphases = 4;
    } else
    {
      frequency = frequency / 3;
      si5351.set_freq(frequency * SI5351_FREQ_MULT, SI5351_CLK0);
      si5351.set_freq((frequency+VNA_NOMINAL_3X_IF_FREQ) * SI5351_FREQ_MULT, SI5351_CLK1);
      numphases = 12;
    }
    return 0;
}

int acquire_dataset(struct _vna_data_set *v, unsigned int timeout)
{
  int n;
  unsigned int freqstep = (vna_endfreq - vna_startfreq) / vna_nfreqs;
  for (n=0;n<vna_nfreqs;n++)
  {
    unsigned int freq = vna_startfreq + freqstep * n;
    setup_frequency_acquire(freq);
    if (n == 0)
    {
       if (acquire_sample(1, timeout) < 0)
       {
         DEBUGMSG("Acquisition aborted");
         return -1;
       }
    }
    if (acquire_sample(vna_num_averages, timeout) < 0)
    {
      DEBUGMSG("Acquisition aborted");
      return -1;
    }
    v->freq[n] = freq;
    v->vna_p1_volt_i[n] = sampVOLTI;
    v->vna_p1_volt_q[n] = sampVOLTQ;
    v->vna_p1_cur_i[n] = sampCURI;
    v->vna_p1_cur_q[n] = sampCURQ;
    v->vna_p2_cur_i[n] = sampCUR2I;
    v->vna_p2_cur_q[n] = sampCUR2Q;
  }
  return 0;
}

int doacq_cmd(int args, tinycl_parameter* tp, void *v)
{
  int n;
  unsigned int timeout = 72000u*tp[0].ti.i;
  unsigned int stt = CPU_CYCLES;
  acquire_dataset(&vna_data,timeout);
  DEBUGMSG("Total cycles=%u",(unsigned int)(CPU_CYCLES-stt));
  for (n=0;n<vna_nfreqs;n++)
  {
    DEBUGMSG("Freq %u Volt (%d,%d) Cur (%d,%d) Cur2 (%d,%d)",vna_data.freq[n],vna_data.vna_p1_volt_i[n],vna_data.vna_p1_volt_q[n],
          vna_data.vna_p1_cur_i[n],vna_data.vna_p1_cur_q[n],vna_data.vna_p2_cur_i[n],vna_data.vna_p2_cur_q[n]);    
  }
}

int setacq_cmd(int args, tinycl_parameter* tp, void *v)
{
  unsigned int mode = tp[0].ti.i;
  unsigned int nfreqs = tp[1].ti.i;
  unsigned int startfreq = tp[2].ti.i;
  unsigned int endfreq = tp[3].ti.i;
  if ((mode >= 1) && (mode <= 3)) vna_mode = mode;
  if ((vna_nfreqs >= 1) && (vna_nfreqs <= VNA_MAX_FREQS)) vna_nfreqs = nfreqs;
  if ((startfreq >= VNA_MIN_FREQ) && (endfreq>startfreq) && (endfreq <= VNA_MAX_FREQ))
  {
    vna_startfreq = startfreq;
    vna_endfreq = endfreq;
  }
  Serial.print("Mode = ");
  Serial.println(vna_mode);
  Serial.print("Start Frequency = ");
  Serial.println(vna_startfreq);
  Serial.print("End Frequency = ");
  Serial.println(vna_endfreq);
  Serial.print("Number Frequencies = ");
  Serial.println(vna_nfreqs);
}

const tinycl_command tcmds[] = 
{
  { "SETACQ", "Set Acquisition Parameters", setacq_cmd, TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_INT },
  { "DOACQ", "Set Acquisition Parameters", doacq_cmd, TINYCL_PARM_INT, TINYCL_PARM_END },
  { "PRINT", "Print something", print_cmd, {TINYCL_PARM_STR, TINYCL_PARM_END } },
  { "PRINT2", "Print 2 somethings", print2_cmd, {TINYCL_PARM_STR, TINYCL_PARM_STR, TINYCL_PARM_END } },
  { "ADD", "Add Two Numbers", add_cmd, {TINYCL_PARM_INT, TINYCL_PARM_INT, TINYCL_PARM_END } },
  { "HELP", "Display This Help", help_cmd, {TINYCL_PARM_END } },
};

int help_cmd(int args, tinycl_parameter *tp, void *v)
{
  tinycl_print_commands(sizeof(tcmds)/sizeof(tinycl_command),tcmds);
}

void loop(void)
{
  if (tinycl_task(sizeof(tcmds)/sizeof(tinycl_command),tcmds,NULL))
    Serial.print("> ");
}

void loop2() {
  static int i = 0;
  if (tinycl_task(sizeof(tcmds)/sizeof(tinycl_command),tcmds,NULL))
    Serial.print("> ");
  delay(100);
  Serial.print("doing something ");
  Serial.print(i++);
  if (current_summed < number_to_sum)
  {
    Serial.println("");
    
    return;
  }
  Serial.print(" ");
  Serial.print(diftick);
  Serial.print(" (");
  for (int j=1;j<numphases;j++)
  {
     if (j != 1) Serial.print(",");
     Serial.print((unsigned int)(timephase[j]-timephase[0]));
  }
  Serial.print(") ");
  Serial.print(curphase);
/*
  Serial.print(" ");
  Serial.print(sampCUR);
  Serial.print(" ");
  Serial.print(sampVOLT);
  Serial.print(" ");
  Serial.println(sampCUR2);
  */
  Serial.print(" (");
  Serial.print(sampCURI);
  Serial.print(",");
  Serial.print(sampCURQ);
  Serial.print(") (");
  Serial.print(sampVOLTI);
  Serial.print(",");
  Serial.print(sampVOLTQ);
  Serial.print(") (");
  Serial.print(sampCUR2I);
  Serial.print(",");
  Serial.print(sampCUR2Q);
  Serial.println(")");
  current_summed = 0;
}

