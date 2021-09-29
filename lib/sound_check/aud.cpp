#include "aud.h"
#include "filt.h"

void aud::rolling(void)
{
    for(size_t l = this->n-1; l>0; l--)
    {
        this->raw[l] = this->raw[l-1];
    }
}

bool aud::update(int val)
{
    
    this->rolling();
    if(this->filter_)
    {
        float alpha_a = getAlpha(micros() - this->time, 3000);
        float alpha_b = getAlpha(micros() - this->time, 1000);
        float filtered_H = ( (0.85*(float)val) + ((1-0.85)*(float)this->last_raw) );
        float filtered_L = ( (0.65*(float)val) + ((1-0.65)*(float)this->last_raw) );
        this->last_raw = val;
        val = (int)(filtered_H - filtered_L);
    };

    this->raw[0] = val;
    this->counter_++;
    if( this->counter_ < this->sampleCounter_ )
        return false;
    else
        this->counter_ = 0;
    double frequency = (double)(micros() - this->time);
    frequency = 1E+6/frequency;
    // Filter *my_filter;
    // my_filter = new Filter(BPF, this->n-1, frequency, 2150.0f, 3355.0f);
    // double out_val;
    
    // for(int i =0; i<this->n; i++)
    // {
    //     out_val = my_filter->do_sample( (double) this->raw[i] );
    //     this->raw[i] = (short)out_val;
    // }
    // delete my_filter;
    // arduinoFFT FFT =arduinoFFT();

    // FFT.Windowing(this->raw, (uint16_t)this->n, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    // //  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
    // FFT.Compute(this->raw, this->mag, (uint16_t)this->n, FFT_FORWARD);
    // // this->FFT.ComplexToMagnitude(this->raw, this->mag, (uint16_t)this->n);
    // // double x = this->FFT.MajorPeak(this->raw,  (uint16_t)this->n, (double)this->fs_);
    
    // if( this->peakDetect() )
        this->FFT(this->raw, this->n, this->fs_, this->nf_);
    // else    
    //     memset(this->f_peaks,0,5);
    // Serial.print(this->fs_);
    // Serial.print(": \t\t");
    // Serial.print("F_peak : ");
    // Serial.print((float)this->f_peaks[0]/1000);
    // Serial.print("\t\t");
    // Serial.print(micros()-time);
    // Serial.print(" us \t\t");
    for(int i = 0; i < this->n; i++)
    {
        if( isnan(this->nf_[i]) || isinf(this->nf_[i]) || (this->nf_[i] > this->fs_) )
            this->nf_[i] = 0;
        Serial.print("raw:");Serial.print(this->raw[i]);
        Serial.print("\tfp:");Serial.println(this->nf_[i]/10.f);
        // Serial.flush();
        this->nf_[i] = 0.f;
        this->raw[i] = 0.f;
    }
    
    this->time = micros();
    return true;
}

float aud::FFT(int in[],int N,float Frequency, float nF[])
{
/*
Code to perform FFT on arduino,
setup:
paste sine_data [91] at top of program [global variable], paste FFT function at end of program
Term:
1. in[]     : Data array, 
2. N        : Number of sample (recommended sample size 2,4,8,16,32,64,128...)
3. Frequency: sampling frequency required as input (Hz)

If sample size is not in power of 2 it will be clipped to lower side of number. 
i.e, for 150 number of samples, code will consider first 128 sample, remaining sample  will be omitted.
For Arduino nano, FFT of more than 128 sample not possible due to mamory limitation (64 recomended)
For higher Number of sample may arise Mamory related issue,
Code by ABHILASH
Contact: abhilashpatel121@gmail.com 
Documentation:https://www.instructables.com/member/abhilash_patel/instructables/
2/3/2021: change data type of N from float to int for >=256 samples
*/

unsigned int data[13]={1,2,4,8,16,32,64,128,256,512,1024,2048};
int a,c1,f,o,x;
a=N;  
                             
      for(int i=0;i<12;i++)                 //calculating the levels
         { if(data[i]<=a){o=i;} }
      
int in_ps[data[o]]={};     //input for sequencing
float out_r[data[o]]={};   //real part of transform
float out_im[data[o]]={};  //imaginory part of transform
           
x=0;  
      for(int b=0;b<o;b++)                     // bit reversal
         {
          c1=data[b];
          f=data[o]/(c1+c1);
                for(int j=0;j<c1;j++)
                    { 
                     x=x+1;
                     in_ps[x]=in_ps[j]+f;
                    }
         }

 
      for(int i=0;i<data[o];i++)            // update input array as per bit reverse order
         {
          if(in_ps[i]<a)
          {out_r[i]=in[in_ps[i]];}
          if(in_ps[i]>a)
          {out_r[i]=in[in_ps[i]-a];}      
         }


int i10,i11,n1;
float e,c,s,tr,ti;

    for(int i=0;i<o;i++)                                    //fft
    {
     i10=data[i];              // overall values of sine/cosine  :
     i11=data[o]/data[i+1];    // loop with similar sine cosine:
     e=360/data[i+1];
     e=0-e;
     n1=0;

          for(int j=0;j<i10;j++)
          {
          c=cosine(e*j);
          s=sine(e*j);    
          n1=j;
          
                for(int k=0;k<i11;k++)
                 {
                 tr=c*out_r[i10+n1]-s*out_im[i10+n1];
                 ti=s*out_r[i10+n1]+c*out_im[i10+n1];
          
                 out_r[n1+i10]=out_r[n1]-tr;
                 out_r[n1]=out_r[n1]+tr;
          
                 out_im[n1+i10]=out_im[n1]-ti;
                 out_im[n1]=out_im[n1]+ti;          
          
                 n1=n1+i10+i10;
                  }       
             }
     }
//---> here onward out_r contains amplitude and our_in conntains frequency (Hz)
    for(int i=0;i<data[o-1];i++)               // getting amplitude from complex number
        {
         out_r[i]=sqrt(out_r[i]*out_r[i]+out_im[i]*out_im[i]); // to  increase the speed delete sqrt
         out_im[i]=i*Frequency/N;   
        }
x=0;       // peak detection
   for(int i=1;i<data[o-1]-1;i++)
      {
      if(out_r[i]>out_r[i-1] && out_r[i]>out_r[i+1]) 
      {in_ps[x]=i;    //in_ps array used for storage of peak number
      x=x+1;}    
      }
s=0;
c=0;
    for(int i=0;i<x;i++)             // re arraange as per magnitude
    {
        for(int j=c;j<x;j++)
        {
            if(out_r[in_ps[i]]<out_r[in_ps[j]]) 
                {s=in_ps[i];
                in_ps[i]=in_ps[j];
                in_ps[j]=s;}
        }
    c=c+1;
    }
    for(int i=0;i<5;i++)     // updating f_peak array (global variable)with descending order
    {
        f_peaks[i]=out_im[in_ps[i]];
    }
    for(int i =0; i<data[o]; i++)
    {
        nF[i] = (float)out_im[in_ps[i]];
        if(isnan(nF[i]) || isinf(nF[i]) || nF[i] < 0.00f)
            nF[i] = 0.f;
    }
    return f_peaks[0];
}

float aud::sine(int i)
{
  int j=i;
  float out;
  while(j<0){j=j+360;}
  while(j>360){j=j-360;}
  if(j>-1   && j<91){out= sine_data[j];}
  else if(j>90  && j<181){out= sine_data[180-j];}
  else if(j>180 && j<271){out= -sine_data[j-180];}
  else if(j>270 && j<361){out= -sine_data[360-j];}
  return (out/255);
}

float aud::cosine(int i)
{
  int j=i;
  float out;
  while(j<0){j=j+360;}
  while(j>360){j=j-360;}
  if(j>-1   && j<91){out= sine_data[90-j];}
  else if(j>90  && j<181){out= -sine_data[j-90];}
  else if(j>180 && j<271){out= -sine_data[270-j];}
  else if(j>270 && j<361){out= sine_data[j-270];}
  return (out/255);
}

bool aud::peakDetect()
{
    int ref1 = abs(this->raw[0] - (int)((float)this->raw[0]*0.1));
    int ref2 = abs(this->raw[1] - (int)((float)this->raw[1]*0.1));
    // ref1 = min(ref1,ref2);
    ref2 = abs(this->raw[1] - (int)((float)this->raw[1]*0.15));
    if( ref1 <  ref2 )
        return false;
    else
    {
        ref2 = abs(this->raw[1] + (int)((float)this->raw[1]*0.15));
        if(ref1>ref2)
            return false;
    }
    return true;
}

float aud::getAlpha(uint32_t time_micros, float fc_)
{
    float a_ = fc_*2*PI*((float)time_micros/1e+6);
	float b_ = 1 + ( 2*PI*fc_* ((float)time_micros/1e+6));
	return (a_/b_);
}