#include "daisy_seed.h"
#include "daisysp.h"
#include <string.h>
#include <math.h>
#include <stdint.h>
#define FILTER_TAP_NUM 65


/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 48000 Hz

* 0 Hz - 18000 Hz
  gain = 1
  desired ripple = 1 dB
  actual ripple = 0.6590242147692962 dB

* 20000 Hz - 24000 Hz
  gain = 0
  desired attenuation = -80 dB
  actual attenuation = -81.14543719476963 dB

*/



static double filter_taps[FILTER_TAP_NUM] = {
  -0.006225687526558968,
  -0.013921944869775747,
  -0.0017033686204005018,
  0.006131027999065173,
  -0.005194914219137801,
  0.0018198750177262203,
  0.0022076255739365237,
  -0.005284487608677204,
  0.005988317243873974,
  -0.0036287395276752214,
  -0.0011512053171730404,
  0.006303260287948152,
  -0.00911536035106789,
  0.007533552368957478,
  -0.001423900950110941,
  -0.006897761224286268,
  0.013376492061692856,
  -0.014039608789908632,
  0.007143638874166622,
  0.005367882547862448,
  -0.01810837120873407,
  0.02425214024584075,
  -0.018714578899311635,
  0.0012037365857198127,
  0.022500345398014286,
  -0.04161545626306594,
  0.044253769077641926,
  -0.022059760293629774,
  -0.025601664867402306,
  0.09020413623043204,
  -0.15605578271365164,
  0.2051306623207623,
  0.7767339681438179,
  0.2051306623207623,
  -0.15605578271365164,
  0.09020413623043204,
  -0.025601664867402306,
  -0.022059760293629774,
  0.044253769077641926,
  -0.04161545626306594,
  0.022500345398014286,
  0.0012037365857198127,
  -0.018714578899311635,
  0.02425214024584075,
  -0.01810837120873407,
  0.005367882547862448,
  0.007143638874166622,
  -0.014039608789908632,
  0.013376492061692856,
  -0.006897761224286268,
  -0.001423900950110941,
  0.007533552368957478,
  -0.00911536035106789,
  0.006303260287948152,
  -0.0011512053171730404,
  -0.0036287395276752214,
  0.005988317243873974,
  -0.005284487608677204,
  0.0022076255739365237,
  0.0018198750177262203,
  -0.005194914219137801,
  0.006131027999065173,
  -0.0017033686204005018,
  -0.013921944869775747,
  -0.006225687526558968
};


using namespace daisy;
using namespace daisysp;
#define MAX_DELAY static_cast<size_t>(48000 * 1.0f)
#define MAX_DELAY1 static_cast<size_t>(48000 * 1.0f)


static DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS del;
static DelayLine<float, MAX_DELAY1> DSY_SDRAM_BSS comb0;
static DelayLine<float, MAX_DELAY1> DSY_SDRAM_BSS comb1;
static DelayLine<float, MAX_DELAY1> DSY_SDRAM_BSS comb2;
static DelayLine<float, MAX_DELAY1> DSY_SDRAM_BSS comb3;
static DelayLine<float, MAX_DELAY1> DSY_SDRAM_BSS allpass0;
static DelayLine<float, MAX_DELAY1> DSY_SDRAM_BSS allpass1;
static DelayLine<float, MAX_DELAY1> DSY_SDRAM_BSS allpass2;
DaisySeed hw;

typedef struct {
	float bufin[2];
	float bufout[2];
	float output;
	float coeff; 
} HP;

typedef struct {
	float b0;
	float b1;
	float a1;
	float K;
	float prev_input;
	float output;
} LP;


typedef struct {

	float buf[65];
	uint8_t bufIndex;

	float out;
} FIRFilter;

//helper decleration
void Init_LPF (float cutoff, LP *lp, float samplefreq);
void Init_HPF (float cutoff, HP *hp, float samplefreq);
float filter_Lp(LP *lp, float input);
float filter_hp(HP *hp, float input);
void FIR_LPF_init (FIRFilter *fir);
float FIRFilter_update(FIRFilter *fir, float input);
float overdrive(float input, float gain);
float new_overdrive(float input, float gain, float threshold);
float do_comb(float input);
float do_allpass(float input);
float reverb(float input, float wet);
float delay(float input, float feedback, float wet);
float delay_time (float knob);
void set_up (float sample_rate);


bool reverb_on = false;
bool overdrive_on = false;
bool delay_on = false;
FIRFilter lpf_in, lpf_out;
HP hp_in;
LP lp_in;
Switch sw_overdrive, sw_delay, sw_reverb; 


void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	float sample_rate=hw.AudioSampleRate();
	sw_overdrive.Debounce();
	sw_delay.Debounce();
	sw_reverb.Debounce();
	//check if any button was pressed
	if (sw_overdrive.RisingEdge())
		overdrive_on=!overdrive_on;
	if (sw_delay.RisingEdge())
		delay_on=!delay_on;
	if (sw_reverb.RisingEdge())
		reverb_on=!reverb_on;
	float signal=0.0f;
	float delay_t = floorf(delay_time(hw.adc.GetFloat(1))*sample_rate);
	if (delay_on)
	{
		del.SetDelay(delay_t);
	}
	
	for (size_t i = 0; i < size; i++)
	{
		//LPF in
		signal=FIRFilter_update(&lpf_in,in[1][i]);
		

		//overdrive
		if(overdrive_on)
		{
			signal = new_overdrive(signal, 10.0f*hw.adc.GetFloat(0) ,3.0f);
		}
	
		//delay
		if (delay_on)
		{
			signal=delay(signal,0.2f, 0.7f);
		}

		//reverb
		if (reverb_on)
		{
			signal=reverb(signal,0.8f*hw.adc.GetFloat(2));
		}

		signal=FIRFilter_update(&lpf_out,signal);

		out[0][i]=signal;
		out[1][i]=signal;
		//out[1][i] = in[1][i];  
		//out[0][i] = in[0][i];
	}
}




int main(void)
{
	AdcChannelConfig par[3];
	hw.Configure();
	hw.Init();
	sw_overdrive.Init(hw.GetPin(18), 1000);
	sw_delay.Init(hw.GetPin(19), 1000);
	sw_reverb.Init(hw.GetPin(20), 1000);
	par[0].InitSingle(hw.GetPin(15));
	par[1].InitSingle(hw.GetPin(16),AdcChannelConfig::SPEED_810CYCLES_5);
	par[2].InitSingle(hw.GetPin(17));
	hw.adc.Init( par, 3);
	hw.adc.Start();
	hw.SetAudioBlockSize(48); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	float sample_rate=hw.AudioSampleRate();
	set_up(sample_rate); // initialized all effects 
	hw.StartAudio(AudioCallback);	
	while(1) {}
}



void FIR_LPF_init (FIRFilter *fir)
{
	for (uint8_t n=0; n<65; n++)
	{
		fir->buf[n]=0.0f;
	}
	fir->bufIndex= 0;
	fir->out=0.0f;
}
float FIRFilter_update(FIRFilter *fir, float input)
{
	//store new sample
	fir->buf[fir->bufIndex]=input;

	//increment buffer index
	fir->bufIndex++;
	if (fir->bufIndex == 65)
	{
		fir->bufIndex= 0;
	}

	//compute the new output
	fir->out= 0.0f;
	uint8_t sumIndex=fir->bufIndex;
	for (uint8_t n = 0; n < 65 ; n++)
	{
		//decrement index and wrap around if needed
		if (sumIndex > 0)
		{
			sumIndex--; 
		}
		else 
		{
			sumIndex=65-1;
		}
		//multply impulse response with shifted input sample and add to output
		fir->out += filter_taps[n]*fir->buf[sumIndex];
	}
	//return filter output
	return (fir->out);
} 

void Init_LPF (float cutoff, LP *lp, float samplefreq)
{
	lp->K=tanf(M_PI * cutoff /samplefreq);
	lp->b0=(lp->K)/((lp->K)+1);
	lp->b1=lp->b0;
	lp->a1=((lp->K)-1)/((lp->K)+1);
}

void Init_HPF (float cutoff, HP *hp, float samplefreq)
{
	hp->coeff=2.0f * M_PI * cutoff/samplefreq; 
	hp->bufin[0]=hp->bufin[1]=0.0f;
	hp->bufout[0]=hp->bufout[1]=0;
}

float filter_lp(LP *lp, float input)
{
	float sig = 0.0f;
	sig = (lp->b0)*input+(lp->b1)*(lp->prev_input)-(lp->a1)*(lp->output);
	lp->prev_input=input;
	lp->output=sig;
	return(sig);
}

float filter_hp(HP *hp, float input)
{
	hp->bufin[1]=hp->bufin[0];
	hp->bufin[0]=input;

	hp->bufout[1]=hp->bufout[0];
	hp->bufout[0]=(2.0f * (hp->bufin[0]-hp->bufin[1]) + (2.0f - hp->coeff)* (hp->bufout[1]) /(2.0f + hp->coeff));
	return (hp->bufout[0]);
}


float new_overdrive(float input, float gain, float threshold)
{
	float sig=0.0f;
	sig=filter_hp(&hp_in, sig); // HPF before overdrive
	sig=gain*input;
	bool sign=false;
	if (sig>0.0f)
	{
		sign=true;
	}

	if(sign)
	{
		if(sig <= 1.0f/threshold)
		{
			sig=sig*(threshold-1.0f);
		}
		else
		{
			if (sig>2.0f*(1.0f/threshold))
			{
				sig=1.0f;
			}
			else
			{
				sig=1.0f-(2.0f -threshold*sig)*(2.0f -threshold *sig)/threshold;
			}
		}
	}
	else
	{
		if(sig >= -1.0f/threshold)
		{
			sig=sig*(threshold-1.0f);
		}
		else
		{
			if (sig<-2.0f*(1.0f/threshold))
			{
				sig=-1.0f;
			}
			else
			{
				sig=-1.0f*(1.0f-(2.0f +threshold *sig)*(2.0f +threshold *sig)/threshold);
			}
		}
	}
	sig=filter_lp(&lp_in, sig); //LPF after clipping
	return (sig/gain);
	
}
float do_comb(float input)
{
	float gain0 = 0.805f, gain1=0.827f, gain2 =0.783f, gain3=0.764f;
	float out0=0.0f,out1=0.0f,out2=0.0f,out3=0.0f;
	//reading delays
	out0=comb0.Read();
	out1=comb1.Read();
	out2=comb2.Read();
	out3=comb3.Read();
	//delay write
	comb0.Write(gain0*out0+input);
	comb1.Write(gain1*out1+input);
	comb2.Write(gain2*out2+input);
	comb3.Write(gain3*out3+input);
	//return total output
	return(0.25f*(out0+out1+out2+out3));
}

float do_allpass(float input)
{
	float gain =0.7f;
	float out0 = 0.0f, out1 = 0.0f, out2 =0.0f;
	//first allpass
	out0 = allpass0.Read()-gain*input;
	allpass0.Write(gain*out0+input);
	//second allpass
	out1=allpass1.Read()-gain*out0;
	allpass1.Write(gain*out1+out0);
	//finall allpass
	out2=allpass2.Read()-gain*out1;
	allpass2.Write(gain*out2+out1);
	//return result
	return (out2);
}

float reverb(float input, float wet)
{
	float signal=0.0f;
	signal=do_comb(input);
	signal=do_allpass(signal);
	return(wet*signal+(1.0f-wet)*input);
}

float delay_time (float knob)
{
	float floor_knob = floorf(knob*1000.0f);
	return (floor_knob/1000.0f);

}

float delay(float input, float feedback, float wet)
{
	float out = del.Read();
	del.Write(input+feedback*out);
	return ((1.0f-wet)*input+wet*out);
}


void set_up (float sample_rate)
{
	
	FIR_LPF_init(&lpf_in);
	FIR_LPF_init(&lpf_out);
	Init_HPF(400.0f ,&hp_in, sample_rate);
	Init_LPF(12000.0f, &lp_in, sample_rate);
	del.Init();
	comb0.Init();
	comb1.Init();
	comb2.Init();
	comb3.Init();
	allpass0.Init();
	allpass1.Init();
	allpass2.Init();
	del.SetDelay(8000.0f);
	//del.SetDelay(sample_rate*0.1f);
	comb0.SetDelay(sample_rate*0.03604f);
	comb1.SetDelay(sample_rate*0.03112f);
	comb2.SetDelay(sample_rate*0.04044f);
	comb3.SetDelay(sample_rate*0.04492f);
	allpass0.SetDelay(sample_rate*0.005f);
	allpass1.SetDelay(sample_rate*0.00168f);
	allpass2.SetDelay(sample_rate*0.00048f);
}
