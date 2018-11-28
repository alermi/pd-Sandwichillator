#include "m_pd.h"
#include <stdlib.h>
#include <math.h>
#ifdef NT
#pragma warning( disable : 4244 )
#pragma warning( disable : 4305 )
#endif

/* ------------------------ sandwichillator~ ----------------------------- */

/*
	This patch builds three waveforms on top of eachother. The harmonics 
	start as triangle wave harmonics, then change to square wave harmonics,
	and then change again to sawtooth wave harmonics.

	This patch is a sonic sandwich!!!

*/
#define WAVETABLESIZE 1024


static t_class *sandwichillator_class;

typedef struct _sandwichillator
{
    t_object x_obj; 	/* obligatory header */
    t_float x_f;    	/* place to hold inlet's value if it's set by message */
	t_float *wavetable;	/* a place to hold the sandwichillatord values */
	t_float phase;
	t_float samplerate;
	//variables that tell how many harmonics of each waveform we want
	t_float saw_harmonics;		
	t_float triangle_harmonics;
	t_float square_harmonics;
	//variables to keep the volume limited
	t_float max_value;
	int harmonics_modified; 
	float volume_limiter;
	int volume_check;
} t_sandwichillator;


static inline float lin_interpolate(t_sandwichillator *x, float harm)
{
	int x_1 = x->phase * WAVETABLESIZE * harm;
	float y_1 = x->wavetable[x_1 % WAVETABLESIZE];
	float y_2 = x->wavetable[(x_1 + 1) % WAVETABLESIZE];

	return (y_2 - y_1) * ((x->phase * WAVETABLESIZE * harm) - x_1) + y_1;
}


static void calculate_samples(t_sandwichillator *x, t_float *freq, t_float *out, int n) {
	int blocksize = n;
	int i, j, k, l, sample = 0;
	float phaseincrement, triflip;
	float findex;
	int	iindex;

	while (n--)
	{
		// first we need to calculate the phase increment from the frequency
		// and sample rate - this is the number of cycles per sample
		// freq = cyc/sec, sr = samp/sec, phaseinc = cyc/samp = freq/sr
		phaseincrement = *(freq + sample) / x->samplerate;

		// now, increment the phase and make sure it doesn't go over 1.0
		x->phase += phaseincrement;
		while (x->phase >= 1.0f)
			x->phase -= 1.0f;
		while (x->phase < 0.0f)
			x->phase += 1.0f;
		// now grab the sample from the table
		findex = WAVETABLESIZE * x->phase;
		iindex = (int)findex;
		*(out + sample) = 0.0f;
		//First, calculate the triangle wave harmonics
		j = 1;
		triflip = 1.0f;
		for (; j < x->triangle_harmonics*2; j += 2) {
			*(out + sample) += (triflip * lin_interpolate(x, (float)j) / (float)(j*j));
			triflip *= -1.0f;
		}
		//Then add the square wave harmonics on top
		for (k=j; k < j+x->square_harmonics*2; k += 2)
		{
			*(out + sample) += (lin_interpolate(x, (float)k) / (float)(k));
		}
		//And then add the sawtooth harmonics.
		for (l=k; l < k+x->saw_harmonics; l += 1) {
			*(out + sample) += (lin_interpolate(x, (float)l) / l);
		}
		//If there are samples left to be checked for volume, check them
		//and find the maximum value among them
		if (x->volume_check > 0) {
			if (*(out + sample) > x->max_value) {
				x->max_value = *(out + sample);
			}
			if (x->volume_check > 0)x->volume_check--;

		}
		else if (x->volume_check == 0 && x->max_value!=0) {
			//Set the limiter value to normalize the volume.
			x->volume_limiter = 1 / x->max_value;
			x->max_value = 0;
		}
		*(out + sample) = *(out + sample)*x->volume_limiter;
		sample++;

	}
}
    /* this is the actual performance routine which acts on the samples.
    It's called with a single pointer "w" which is our location in the
    DSP call list.  We return a new "w" which will point to the next item
    after us.  Meanwhile, w[0] is just a pointer to dsp-perform itself
    (no use to us), w[1] and w[2] are the input and output vector locations,
    and w[3] is the number of points to calculate. */
static t_int *sandwichillator_perform(t_int *w)
{
	t_sandwichillator *x = (t_sandwichillator *)(w[1]);
    t_float *freq = (t_float *)(w[2]);
    t_float *out = (t_float *)(w[3]);
    int n = (int)(w[4]);
	//if the harmonic numbers have been modified, set the number of samples
	//that you need to keep track of the max value (one full cycle of the table).
	if (x->harmonics_modified) { 
		//number of samples it is going to take to go through the 
		//whole wavetable
		x->volume_check = x->samplerate / *(freq);
		x->harmonics_modified = 0;
	}
	//Call the method to calculate the samples
	calculate_samples(x, freq, out, n);
	
    return (w+5);
}

    /* called to start DSP.  Here we call Pd back to add our perform
    routine to a linear callback list which Pd in turn calls to grind
    out the samples. */
static void sandwichillator_dsp(t_sandwichillator *x, t_signal **sp)
{
	// we'll initialize samplerate when starting up
	x->samplerate = sp[0]->s_sr;
	//assume the harmonics have been modified when dsp turns on
	x->harmonics_modified = 1; 
	//passive limiter, doesnt work at 1
	x->volume_limiter = 1;
    dsp_add(sandwichillator_perform, 4, x, sp[0]->s_vec, sp[1]->s_vec, sp[0]->s_n);
}



static void sandwichillator_free(t_sandwichillator *x)
{
	free(x->wavetable);
}

//Input methods that set the variables for harmonic numbers of each waveform
static void sandwichillator_saw_modify(t_sandwichillator *x, t_floatarg g) {
	//If the harmonics have been modified, we have to check the volume level
	x->harmonics_modified = 1;
	if (g > 0) {
		x->saw_harmonics = g;
	}
	else {
		x->saw_harmonics = 0;
	}
}
static void sandwichillator_triangle_modify(t_sandwichillator *x, t_floatarg g) {
	//If the harmonics have been modified, we have to check the volume level
	x->harmonics_modified = 1;
	if (g > 0) {
		x->triangle_harmonics = g;
	}
	else {
		x->triangle_harmonics = 0;
	}
}
static void sandwichillator_square_modify(t_sandwichillator *x, t_floatarg g) {
	//If the harmonics have been modified, we have to check the volume level
	x->harmonics_modified = 1;
	if (g > 0) {
		x->square_harmonics = g;
	}
	else {
		x->square_harmonics = 0;
	}
}
static void *sandwichillator_new(void)
{
	float twopi, size;
	int i;
	t_sandwichillator *x = (t_sandwichillator *)pd_new(sandwichillator_class);
	outlet_new(&x->x_obj, gensym("signal"));
	// initialize variables
	x->x_f = 0.0f;
	x->phase = 0.0f;
	twopi = 8.0f * atanf(1.0f);
	size = (float)WAVETABLESIZE;

	x->wavetable = (t_float *)malloc(WAVETABLESIZE * sizeof(t_float));

	// fill it up with a sine wave
	for (i = 0; i < WAVETABLESIZE; i++)
		*(x->wavetable + i) = sinf(twopi * (float)i / size);

	x->harmonics_modified = 1;
	//passive limiter, doesnt work at 1
	x->volume_limiter = 1;
	//set the defaults of harmonic numbers
	x->saw_harmonics = 0;
	x->square_harmonics = 0;
	x->triangle_harmonics = 0;
	inlet_new(&x->x_obj, &x->x_obj.ob_pd, gensym("float"), gensym("ft1"));
	inlet_new(&x->x_obj, &x->x_obj.ob_pd, gensym("float"), gensym("ft2"));
	inlet_new(&x->x_obj, &x->x_obj.ob_pd, gensym("float"), gensym("ft3"));
	return (x);
}
    /* this routine, which must have exactly this name (with the "~" replaced
    by "_tilde) is called when the code is first loaded, and tells Pd how
    to build the "class". */
void sandwichillator_tilde_setup(void)
{
    sandwichillator_class = class_new(gensym("sandwichillator~"), (t_newmethod)sandwichillator_new, (t_method)sandwichillator_free,
    	sizeof(t_sandwichillator), 0, A_DEFFLOAT, 0);   
    CLASS_MAINSIGNALIN(sandwichillator_class, t_sandwichillator, x_f);
    class_addmethod(sandwichillator_class, (t_method)sandwichillator_dsp, gensym("dsp"), 0);
	class_addmethod(sandwichillator_class, (t_method)sandwichillator_saw_modify, gensym("ft3"), A_FLOAT, 0);
	class_addmethod(sandwichillator_class, (t_method)sandwichillator_triangle_modify, gensym("ft1"), A_FLOAT, 0);
	class_addmethod(sandwichillator_class, (t_method)sandwichillator_square_modify, gensym("ft2"), A_FLOAT, 0);
}
