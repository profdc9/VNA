#ifndef _VNA_H
#define _VNA_H

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

#include "complex.h"

#define VNA_MAX_CAL_FREQS 80
#define VNA_MAX_ACQ_FREQS 100
#define VNA_MIN_FREQ 30000u
#define VNA_MAX_FREQ 1200000000u
#define VNA_FREQ_3X 200000000u
#define VNA_FREQ_5X 600000000u
#define VNA_FREQ_7X 1000000000u
#define VNA_NOMINAL_1X_IF_FREQ 9011u
#define VNA_NOMINAL_3X_IF_FREQ (VNA_NOMINAL_1X_IF_FREQ/3)
#define VNA_NOMINAL_5X_IF_FREQ (VNA_NOMINAL_1X_IF_FREQ/5)
#define VNA_NOMINAL_7X_IF_FREQ (VNA_NOMINAL_1X_IF_FREQ/7)

#define VNA_TOUCHSCREEN

typedef struct _vna_flash_header
{
  unsigned int flash_header_1;
  unsigned int flash_header_2;
} vna_flash_header;

extern const vna_flash_header flash_header;

#define VNA_NO_CALIB        0x00
#define VNA_OPEN_CALIB      0x01
#define VNA_SHORT_CALIB     0x02
#define VNA_LOAD_CALIB      0x04
#define VNA_ALL_1PT         (VNA_OPEN_CALIB|VNA_SHORT_CALIB|VNA_LOAD_CALIB)
#define VNA_VALID_CALIB_1PT 0x08
#define VNA_VALID_CALIB_2PT 0x10

typedef struct _vna_acquisition_state
{
  unsigned int cal_startfreq;
  unsigned int cal_endfreq;
  unsigned int startfreq;
  unsigned int endfreq;
  unsigned short char_impedance;
  unsigned short num_averages;
  unsigned short timeout;
  unsigned short cal_nfreqs;
  unsigned short nfreqs;
  unsigned char csv;
  unsigned char atten;
  unsigned char series_shunt_two;
  unsigned char remote;
  unsigned short calib_state;
} vna_acquisition_state;

extern vna_acquisition_state vna_state;

typedef struct _vna_calib_freq_parm
{
  Complex b, c, d;
  Complex z2, i2, s2;
} vna_calib_freq_parm;

typedef struct _vna_calib_oneport
{
  Complex zo, zs, zt;
} vna_calib_oneport;

extern vna_calib_oneport *vna_1pt;

extern vna_calib_freq_parm vna_calib[];

void reset_calib_state(void);
int acquire_sample(unsigned int averages, unsigned int timeout);
int setup_frequency_acquire(unsigned int frequency);

typedef struct _vna_acquire_dataset_state
{
  unsigned int n;
  unsigned int total;
  unsigned int freq;
  int volti;
  int voltq;
  int curi;
  int curq;
  int cur2i;
  int cur2q;  
} vna_acquire_dataset_state;

typedef int (*vna_acquire_dataset_operation)(vna_acquire_dataset_state *vads, void *v);
typedef int (*vna_report_trans_reflected)(int n, int total, unsigned int, bool ch2, Complex trans, Complex ref);

bool vna_acquire_dataset(vna_acquisition_state *vs, vna_acquire_dataset_operation vado, void *v);

typedef enum _vna_acquire_current_state { VNA_ACQUIRE_COMPLETE=0, VNA_ACQUIRE_ABORT, VNA_ACQUIRE_TIMEOUT, VNA_ACQUIRE_RECORDED_STATE, VNA_ACQUIRE_RECORDED_STATE_WAITING } vna_acquire_current_state;

typedef struct _vna_acquire_state
{
  vna_acquisition_state        *vs;
  bool                          wait_for_end;
  bool                          check_interruption;
  unsigned int                  freqstep;
  unsigned int                  timeout;
  unsigned int                  inittime;
  int                           n;
  vna_acquire_current_state     vacs;
  vna_acquire_dataset_operation vado;
  void                         *vado_v;
} vna_acquire_state;

void printfloat(float f);
int vna_writecal(int n);
int vna_readcal(int n);
int vna_calentry(int n, char *s, int len);
extern vna_acquisition_state vna_state;
int vna_loadcalib(void);
int vna_shortcalib(void);
int vna_opencalib(void);
int vna_thrucal(void);
int vna_set_frequencies(unsigned int nfreqs, unsigned int startfreq, unsigned int endfreq);
int vna_acquire_sparm(vna_report_trans_reflected vrtr);
int vna_acquire_impedance(vna_report_trans_reflected vrtr);
int vna_set_averages(unsigned short averages, unsigned short timeout);
int vna_set_characteristic_impedance(unsigned int char_impedance);
void  vna_initialize_si5351(void);
void vna_setup_remote_serial(void);
int vna_calset_frequencies(unsigned int nfreqs, unsigned int startfreq, unsigned int endfreq);
int vna_display_sparm_operation(vna_acquire_dataset_state *vads, void *va);
int vna_display_acq_operation(vna_acquire_dataset_state *vads, void *va);
vna_acquire_current_state vna_setup_acquire_dataset(vna_acquire_state *vas, vna_acquisition_state *vs, vna_acquire_dataset_operation vado, void *v);
vna_acquire_current_state vna_operation_acquire_dataset(vna_acquire_state *vas);
int vna_mass_storage_device(void);

#endif  /* _VNA_H */
