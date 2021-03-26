/******************************************************************************
 *  FILE
 *      vf_pwm.c
 *
 *  DESCRIPTION
 *      This file provides the implementation of the Very Fast PWM library.
 *      The PWM output has a frequency of 15686 Hz, and the pulse widths 
 *      can be adjusted in 0.25 microsecond steps. Up to 4 PWM outputs
 *      are available. It's possible to have more PWM outputs (up to 8)
 *      but not implemented in this release.
 *
 *      The PIO Controller code is generated on the fly according to the 
 *      patterns specified by the user. For 4 PWM outputs, at most 0x2e
 *      bytes will be generated. An example listing file named 
 *      pio_ctrlr_code.lst can be found in the source folder.
 *
 *  NOTICE
 *      Use of beta release software is at your own risk.  This software is
 *      provided "as is," and CSR cautions users to determine for themselves
 *      the suitability of using the beta version of this software.  CSR
 *      makes no warranty or representation whatsoever of merchantability or
 *      fitness of the product for any particular purpose or use.  In no
 *      event shall CSR be liable for any consequential, incidental or
 *      special damages whatsoever arising out of the use of or inability to
 *      use this software, even if the user has advised CSR of the
 *      possibility of such damages.  Provision of this software does not
 *      imply that it has been given a Bluetooth Qualification listing.
 *      There is no guarantee that CSR will seek a Bluetooth Qualification
 *      listing for the software.
 *
 *****************************************************************************/

#define MAX_LENGTH_IN_BYTES 0x2e

#include <sleep.h>
#include <pio_ctrlr.h>

#include "vf_pwm.h"

typedef struct 
{
	uint16 *buffer;
	uint16 index;
	uint16 length; /* in bytes */
} pio_ctrlr_code_buffer;

static uint16 pwm_pio_reg;
static uint16 pwm_pio_drive_reg;
static uint16 pwm0_pio_mask;
static uint16 pwm1_pio_mask;
static bool pwm_inverted;

static void init_buffer(pio_ctrlr_code_buffer*p, uint16 *buffer, uint16 length)
{
	uint16 i;
	p->buffer = buffer;
	p->length = length;
	p->index = 2; /* the first word is for size (in byte) */
	length = (length>>1)+(length&1);
	for(i=0;i<length;i++) buffer[i]=0;
}

static bool write_buffer(pio_ctrlr_code_buffer*p, uint16 *buffer, uint16 length)
{
	uint16 data;
	uint16 i;
	for(i=0; i<length; i++)
	{
		if(p->index>=p->length) return 0;
		if(i&1) data = (buffer[i>>1]&0xff00)>>8;
		else data = buffer[i>>1]&0xff;
		if(p->index&1) 
        {
            uint16 temp=p->buffer[p->index>>1];
            temp=(temp&0x00ff);
            p->buffer[p->index>>1]=temp|(data<<8);
        }
		else
        {
            uint16 temp=p->buffer[p->index>>1];
            temp=(temp&0xff00);
            p->buffer[p->index>>1]=temp|data;
        }
		p->index++;
	}
	return 1;
}

static bool write_header(pio_ctrlr_code_buffer*p)
{
	uint16 header[4];

    header[0]=(pwm_pio_reg<<8)|0x79; /* mov r1, Px */
	header[1]=0x0178; /* mov r0, 1 */
	header[2]=0x75|(pwm_pio_drive_reg<<8); /* mov Px_DRIVE_EN, mask */
    header[3]=pwm0_pio_mask|pwm1_pio_mask;

    return write_buffer(p, header, 7);
}

static bool write_output(pio_ctrlr_code_buffer*p, uint16 pattern, uint16 count)
{
	uint16 temp[2];

    if(pwm_inverted) pattern = (~pattern)&0xff;

    temp[0]=(pattern<<8)|0x77; /* mov @r1, pattern */
	temp[1]=(count<<8)|0x74; /* mov a, count */

    return write_buffer(p, temp, 4);
}

static bool write_loop(pio_ctrlr_code_buffer*p)
{
	static uint16 loop[2]={
		0x7098, /* subb a, r0 */
		0x00fd  /* jnz 0xfd */
	};

    return write_buffer(p, loop, 3);
}

static bool write_end(pio_ctrlr_code_buffer*p, uint16 pattern)
{
	uint16 temp[2];

    if(pwm_inverted) pattern = (~pattern)&0xff;

    temp[0]=(pattern<<8)|0x77;  /* mov @r1, pattern */
	temp[1]=0x0701; /* ajmp 0x07 */

    if(write_buffer(p, temp, 4))
	{
		p->buffer[0]=p->index-2; /* number of code bytes */
        if(p->buffer[0]&1) /* PioCtrlrInit requires an even number here */
            p->buffer[0]++;
		return 1;
	}
	else
		return 0;
}

bool VFPWMConfig(uint8 pio_bank, uint8 pio_bit0, uint8 pio_bit1,bool invert)
{
	switch(pio_bank)
	{
	case 0:
		pwm_pio_reg=0x80;
		pwm_pio_drive_reg=0xc0;
		break;
	case 1:
		pwm_pio_reg=0x90;
		pwm_pio_drive_reg=0xc8;
		break;
	case 2:
		pwm_pio_reg=0xa0;
		pwm_pio_drive_reg=0xd8;
		break;	
	case 3:
		pwm_pio_reg=0xb0;
		pwm_pio_drive_reg=0xe8;
		break;
	default:
		return 0;
	}

	pwm0_pio_mask=1L<<pio_bit0;
	pwm1_pio_mask=1L<<pio_bit1;

    
    pwm_inverted = invert;

    SleepModeChange(sleep_mode_shallow);
	PioCtrlrClock(TRUE);
    
	return 1;
}

bool VFPWMStart(uint8 width0, uint8 width1)
{
	uint16 buffer[1+MAX_LENGTH_IN_BYTES/2];
	pio_ctrlr_code_buffer cb;
	uint16 counter, previous_counter=0;
	uint16 pattern = pwm0_pio_mask|pwm1_pio_mask;

	init_buffer(&cb, buffer, 2*(sizeof buffer)/sizeof buffer[0]);

	if(!write_header(&cb)) return 0;
	
	for(counter=0; counter<255; counter++)
	{
		if(counter==width0 ||
		counter==width1 ||
		counter==254)
		{
			if(counter>0) {
               
				uint16 gap=counter-previous_counter;
				if(!write_output(&cb, pattern, gap-1))return 0;
                
				if(gap>1)
				if(!write_loop(&cb))return 0;
                
			}
          
			pattern = 0;
			if(width0 > counter) pattern |= pwm0_pio_mask;
			if(width1 > counter) pattern |= pwm1_pio_mask;
			
			previous_counter = counter;
		} else if(counter==0)
		{
			previous_counter = counter;
		}
	}
   
	if(!write_end(&cb, pattern)) return 0;
    
    PioCtrlrStop();
	PioCtrlrInit(cb.buffer-0x2000); /* This function expects code from CODE space */
    PioCtrlrStart();
    return 1;
   
}
void VFPWMStop(void)
{
	PioCtrlrStop();
}
