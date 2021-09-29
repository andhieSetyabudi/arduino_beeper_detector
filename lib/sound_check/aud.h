#ifndef AUD_H_
#define AUD_H_

#include "Arduino.h"


class aud{
    private :
        int* raw;
        int last_raw;
        float *nf_;
        bool filter_;
        size_t n;
        uint8_t exponent_;
        uint16_t sampleCounter_;
        uint16_t counter_;
        float fs_;
        unsigned long time = 0;

        byte sine_data [91]=
        {
            0,  
            4,    9,    13,   18,   22,   27,   31,   35,   40,   44, 
            49,   53,   57,   62,   66,   70,   75,   79,   83,   87, 
            91,   96,   100,  104,  108,  112,  116,  120,  124,  127,  
            131,  135,  139,  143,  146,  150,  153,  157,  160,  164,  
            167,  171,  174,  177,  180,  183,  186,  189,  192,  195,       //Paste this at top of program
            198,  201,  204,  206,  209,  211,  214,  216,  219,  221,  
            223,  225,  227,  229,  231,  233,  235,  236,  238,  240,  
            241,  243,  244,  245,  246,  247,  248,  249,  250,  251,  
            252,  253,  253,  254,  254,  254,  255,  255,  255,  255
        };
        float f_peaks[5]; // top 5 frequencies peaks in descending order

        void rolling(void);
        float cosine(int i);
        float sine(int i);
        float FFT(int in[],int N,float Frequency, float nF[]);
        bool peakDetect();
        float getAlpha(uint32_t time_, float fc_);
    public  :
        aud(size_t size_array, float fs = 3250, bool filter = false):n(size_array),fs_(fs),filter_(filter){};
        bool init()
        {
            this->raw = (int*)malloc(this->n * sizeof(int));
            if(this->raw == NULL)
                return false;          
            this->nf_ = (float*)malloc(this->n * sizeof(float));
            if(this->nf_ == NULL)
                return false;    
            memset(this->raw, 0, this->n);
            this->sampleCounter_ = (uint16_t) ((float)this->n / 2.f);
            if( this->sampleCounter_ < 1 )
                this->sampleCounter_ = 1;
            this->counter_ = 0;
            this->last_raw = 0;
            return true;
        };
        bool update(int val);

};
#endif