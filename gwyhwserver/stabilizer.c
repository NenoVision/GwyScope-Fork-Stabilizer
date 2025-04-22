#include <math.h>
#include <pthread.h>
#include <string.h>

#include "data.h"
#include "stabilizer.h"


static double stbl_read_channel(HWData* hw, const char* chan) {
    double x, y, result;
    pthread_rwlock_rdlock(&hw->cardmutex);

    if (hw->x_external) {
        x = (hw->in[hw->x_external_source] + hw->x_external_offset)*hw->x_external_slope;
    } else {
        x = hw->xpos;
    }

    if (hw->y_external) {
        y = (hw->in[hw->y_external_source] + hw->y_external_offset)*hw->y_external_slope;
    } else {
        y = hw->ypos;
    }

    if (strcmp(chan, "x") == 0) result =  x;
    else if (strcmp(chan, "y") == 0) result =  y;
    else if (strcmp(chan, "z") == 0) result =  hw->zpiezo;
    else if (strcmp(chan, "e") == 0) result =  hw->errorsignal;
    else if (strcmp(chan, "ts") == 0) result =  hw->timestamp;
    else if (strcmp(chan, "adc1") == 0) result =  hw->adc1;
    else if (strcmp(chan, "adc2") == 0) result =  hw->adc2;
    else if (strcmp(chan, "a1") == 0) result =  hw->amplitude1;
    else if (strcmp(chan, "p1") == 0) result =  hw->phase1;
    else if (strcmp(chan, "a2") == 0) result =  hw->amplitude2;
    else if (strcmp(chan, "p2") == 0) result =  hw->phase2;
    else if (strcmp(chan, "in1") == 0) result =  hw->in[0];
    else if (strcmp(chan, "in2") == 0) result =  hw-> in[1];
    else if (strcmp(chan, "in3") == 0) result =  hw->in[2];
    else if (strcmp(chan, "in4") == 0) result =  hw->in[3];
    else if (strcmp(chan, "in5") == 0) result =  hw->in[4];
    else if (strcmp(chan, "in6") == 0) result =  hw->in[5];
    else if (strcmp(chan, "in7") == 0) result =  hw->in[6];
    else if (strcmp(chan, "in8") == 0) result =  hw->in[7];
    else if (strcmp(chan, "in9") == 0) result =  hw->in[8];
    else if (strcmp(chan, "in10") == 0) result =  hw->in[9];
    else if (strcmp(chan, "in11") == 0) result =  hw->in[10];
    else if (strcmp(chan, "in12") == 0) result =  hw->in[11];
    else if (strcmp(chan, "in13") == 0) result =  hw->in[12];
    else if (strcmp(chan, "in14") == 0) result =  hw->in[13];
    else if (strcmp(chan, "in15") == 0) result =  hw->in[14];
    else if (strcmp(chan, "in16") == 0) result =  hw->in[15];
    else if (strcmp(chan, "fmdrive") == 0) result =  hw->amresult;
    else if (strcmp(chan, "kpfm") == 0) result =  hw->f2_offset;
    else if (strcmp(chan, "dart") == 0) result =  hw->dart_frequency + hw->dart_frequencyshift;
    else if (strcmp(chan, "l1x") == 0) result =  hw->l1x;
    else if (strcmp(chan, "l1y") == 0) result =  hw->l1y;
    else if (strcmp(chan, "l2x") == 0) result =  hw->l2x;
    else if (strcmp(chan, "l2y") == 0) result =  hw->l2y;
    else {
        printf("Unknown setpoint stabilization channel\n");
    }

    pthread_rwlock_unlock(&hw->cardmutex);
    return result;
}

static void set_freq(HWData* hw) {
   librp_SetGen(&hw->rp, LIBRP_CH_1, hw->f1_frequency, hw->f1_amplitude, hw->f1_offset);
}

static void stbl_write(HWData* hw, const char* chan, double value) {
    pthread_rwlock_wrlock(&hw->controlmutex);

    if(strcmp(chan,"out1") == 0) hw->aout[0] = value;
    else if(strcmp(chan,"out2") == 0) hw->aout[1] = value;
    else if(strcmp(chan,"out3") == 0) hw->aout[2] = value;
    else if(strcmp(chan,"out4") == 0) hw->aout[3] = value;
    else if(strcmp(chan,"out5") == 0) hw->aout[4] = value;
    else if(strcmp(chan,"out6") == 0) hw->aout[5] = value;
    else if(strcmp(chan,"out7") == 0) hw->aout[6] = value;
    else if(strcmp(chan,"out8") == 0) hw->aout[7] = value;
    else if(strcmp(chan,"out9") == 0) hw->aout[8] = value;
    else if(strcmp(chan,"out10") == 0) hw->aout[9] = value;
    else if(strcmp(chan,"out11") == 0) hw->aout[10] = value;
    else if(strcmp(chan,"out12") == 0) hw->aout[11] = value;
    else if(strcmp(chan,"out13") == 0) hw->aout[12] = value;
    else if(strcmp(chan,"out14") == 0) hw->aout[13] = value;
    else if(strcmp(chan,"out15") == 0) hw->aout[14] = value;
    else if(strcmp(chan,"out16") == 0) hw->aout[15] = value;
    else if(strcmp(chan,"freq1_f") == 0) {
        hw->f1_frequency = value;
        set_freq(hw);
    } else if(strcmp(chan,"freq1_a") == 0) {
        hw->f1_amplitude = value;
        set_freq(hw);
    } else if(strcmp(chan,"freq1_o") == 0) {
        hw->f1_offset = value;
        set_freq(hw);
    }
    else {
        printf("Unknown read out channel\n");
    }

    pthread_rwlock_unlock(&hw->controlmutex);
}

static double stbl_read(HWData* hw, const char* chan) {
    double result;

    pthread_rwlock_wrlock(&hw->controlmutex);

    if(strcmp(chan,"out1") == 0) result = hw->aout[0];
    else if(strcmp(chan,"out2") == 0) result = hw->aout[1];
    else if(strcmp(chan,"out3") == 0) result = hw->aout[2];
    else if(strcmp(chan,"out4") == 0) result = hw->aout[3];
    else if(strcmp(chan,"out5") == 0) result = hw->aout[4];
    else if(strcmp(chan,"out6") == 0) result = hw->aout[5];
    else if(strcmp(chan,"out7") == 0) result = hw->aout[6];
    else if(strcmp(chan,"out8") == 0) result = hw->aout[7];
    else if(strcmp(chan,"out9") == 0) result = hw->aout[8];
    else if(strcmp(chan,"out10") == 0) result = hw->aout[9];
    else if(strcmp(chan,"out11") == 0) result = hw->aout[10];
    else if(strcmp(chan,"out12") == 0) result = hw->aout[11];
    else if(strcmp(chan,"out13") == 0) result = hw->aout[12];
    else if(strcmp(chan,"out14") == 0) result = hw->aout[13];
    else if(strcmp(chan,"out15") == 0) result = hw->aout[14];
    else if(strcmp(chan,"out16") == 0) result = hw->aout[15];
    else if(strcmp(chan,"freq1_f") == 0) result = hw->f1_frequency;
    else if(strcmp(chan,"freq1_a") == 0) result = hw->f1_amplitude;
    else if(strcmp(chan,"freq1_o") == 0) result = hw->f1_offset;
    else {
        printf("Unknown write out channel\n");
    }

    pthread_rwlock_unlock(&hw->controlmutex);

    return result;
}

static void stbl_clear_iteration(HWData* hw) {
    Stabilizer* stbl = hw->stabilizer;
    stbl->sum = 0;
    stbl->count = 0;
}

static bool stbl_collect(HWData* hw) {
    Stabilizer* stbl = hw->stabilizer;
    bool isDone = FALSE;

    if (stbl->count < stbl->oversampling) {
        stbl->sum += stbl_read_channel(hw, stbl->tuning_param);
        stbl->count++;
    } else {
        // divide by count, because count has the actual number of collected samples,
        // and not by oversampling, because it might be changed by the user
        // (eg from 20 -> 10 but what if we had 15 samples already)
        stbl->mean = stbl->sum / stbl->count;
        isDone = TRUE;
    }

    return isDone;
}

static bool stbl_stabilize(HWData* hw) {
    Stabilizer* stbl = hw->stabilizer;
    bool isDone = TRUE;
    double error = stbl->mean - stbl->setpoint;

    if (fabs(error) > stbl->control_range) {
        double step = error > 0 ? -stbl->step : stbl->step;
        double value = stbl_read(hw, stbl->control_output) + step;
        stbl_write(hw, stbl->control_output, value);
        stbl->adjusted = TRUE;
        isDone = FALSE;
    }

    return isDone;
}

static bool stbl_is_control_output_valid(const char* chan) {
    if (strcmp(chan,"out1") == 0) return TRUE;
    else if (strcmp(chan,"out2") == 0) return TRUE;
    else if (strcmp(chan,"out3") == 0) return TRUE;
    else if (strcmp(chan,"out4") == 0) return TRUE;
    else if (strcmp(chan,"out5") == 0) return TRUE;
    else if (strcmp(chan,"out6") == 0) return TRUE;
    else if (strcmp(chan,"out7") == 0) return TRUE;
    else if (strcmp(chan,"out8") == 0) return TRUE;
    else if (strcmp(chan,"out9") == 0) return TRUE;
    else if (strcmp(chan,"out10") == 0) return TRUE;
    else if (strcmp(chan,"out11") == 0) return TRUE;
    else if (strcmp(chan,"out12") == 0) return TRUE;
    else if (strcmp(chan,"out13") == 0) return TRUE;
    else if (strcmp(chan,"out14") == 0) return TRUE;
    else if (strcmp(chan,"out15") == 0) return TRUE;
    else if (strcmp(chan,"out16") == 0) return TRUE;
    else if (strcmp(chan,"freq1_f") == 0) return TRUE;
    else if (strcmp(chan,"freq1_a") == 0) return TRUE;
    else if (strcmp(chan,"freq1_o") == 0) return TRUE;
    else {
        printf("%s is not valid control output for setpoint stabilization\n", chan);
        return FALSE;
    }
}

static bool stbl_is_tuning_param_valid(const char* chan) {
    if (strcmp(chan, "x") == 0) return TRUE ;
    else if (strcmp(chan, "y") == 0) return TRUE ;
    else if (strcmp(chan, "z") == 0) return TRUE;
    else if (strcmp(chan, "e") == 0) return TRUE;
    else if (strcmp(chan, "ts") == 0) return TRUE;
    else if (strcmp(chan, "adc1") == 0) return TRUE;
    else if (strcmp(chan, "adc2") == 0) return TRUE;
    else if (strcmp(chan, "a1") == 0) return TRUE;
    else if (strcmp(chan, "p1") == 0) return TRUE;
    else if (strcmp(chan, "a2") == 0) return TRUE;
    else if (strcmp(chan, "p2") == 0) return TRUE;
    else if (strcmp(chan, "in1") == 0) return TRUE;
    else if (strcmp(chan, "in2") == 0) return TRUE;
    else if (strcmp(chan, "in3") == 0) return TRUE;
    else if (strcmp(chan, "in4") == 0) return TRUE;
    else if (strcmp(chan, "in5") == 0) return TRUE;
    else if (strcmp(chan, "in6") == 0) return TRUE;
    else if (strcmp(chan, "in7") == 0) return TRUE;
    else if (strcmp(chan, "in8") == 0) return TRUE;
    else if (strcmp(chan, "in9") == 0) return TRUE;
    else if (strcmp(chan, "in10") == 0) return TRUE;
    else if (strcmp(chan, "in11") == 0) return TRUE;
    else if (strcmp(chan, "in12") == 0) return TRUE;
    else if (strcmp(chan, "in13") == 0) return TRUE;
    else if (strcmp(chan, "in14") == 0) return TRUE;
    else if (strcmp(chan, "in15") == 0) return TRUE;
    else if (strcmp(chan, "in16") == 0) return TRUE;
    else if (strcmp(chan, "fmdrive") == 0) return TRUE;
    else if (strcmp(chan, "kpfm") == 0) return TRUE;
    else if (strcmp(chan, "dart") == 0) return TRUE;
    else if (strcmp(chan, "l1x") == 0) return TRUE;
    else if (strcmp(chan, "l1y") == 0) return TRUE;
    else if (strcmp(chan, "l2x") == 0) return TRUE;
    else if (strcmp(chan, "l2y") == 0) return TRUE;
    else {
        printf("%s is not valid tuning param for setpoint stabilization\n", chan);
        return FALSE;
    }
}

static bool stbl_start_iteration(HWData* hw) {
    Stabilizer* stbl = hw->stabilizer;
    bool started = FALSE;

    if (stbl_is_tuning_param_valid(stbl->tuning_param) && stbl_is_control_output_valid(stbl->control_output)) {
        stbl_clear_iteration(hw);
        started = TRUE;
        stbl->mode = STBL_COLLECT;
        stbl->is_active = TRUE;
    } else {
        printf("Do not start Setpoint stabilization\n");
        stbl_stop(hw);
    }

    return started;
}


bool stbl_start(HWData* hw) {
    Stabilizer* stbl = hw->stabilizer;
    stbl->adjusted = FALSE;
    return stbl_start_iteration(hw);
}

void stbl_stop(HWData* hw) {
    Stabilizer* stbl = hw->stabilizer;
    stbl->is_active = FALSE;
    stbl->mode = STBL_FINISHED;
    stbl_clear_iteration(hw);
}

void stbl_run(HWData* hw) {
    Stabilizer* stbl = hw->stabilizer;

    if (stbl->mode == STBL_COLLECT) {
        bool isDone = stbl_collect(hw);
        if (isDone) {
            stbl->mode = STBL_STABILIZE;
        }
    } else if (stbl->mode == STBL_STABILIZE) {
        bool isDone = stbl_stabilize(hw);
        if (isDone) {
            stbl_stop(hw);
        } else {
            stbl_start_iteration(hw);
        }
    }
}
