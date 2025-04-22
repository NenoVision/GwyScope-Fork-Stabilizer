
#include <math.h>

#include "data.h"
#include "fake.h"

static int get_index_from_position(double pos, double range, int n_points) {
    double rel_pos = pos / range;
    return round(n_points * rel_pos);
}

static double generate_fake_point(double x, double y) {
    double tx = (x * 0.91 - y * 0.3) / 128;
    double ty = (x * 0.3 + y * 0.91) / 128;
    int txx = (int)tx & 1;
    int tyy = (int)ty & 1;
    int grid = pow(20, txx + tyy);
    double ridge = pow(x - y * 0.8, 2) * 0.05 - 4;
    double wave =
        sin(x * 0.45 - y * 0.15) * sin(x * 0.15 + y * 0.45) * sin((x - y) * 0.01);
    double min = grid < ridge ? grid : ridge;
    return min + y + wave * 5;
}

static double generate_fake_z_point(double xpos, double ypos, double xrange, double yrange) {
    int x = get_index_from_position(xpos, xrange, 2000 - 1);
    int y = get_index_from_position(ypos, yrange, 2000 - 1);

    double center = generate_fake_point(x, y);
    double top = generate_fake_point(x - 1, y);
    double bottom = generate_fake_point(x + 1, y);
    double left = generate_fake_point(x, y - 1);
    double right = generate_fake_point(x, y + 1);

    center += top + bottom + left + right;
    center *= 0.111;
    center /= 1e9;

    return center;
}

int store_fake_point_data(HWData *hw, int set) {
   int i;

   if (hw->scan_ndata >= hw->allocated_scan_ndata) return hw->scan_ndata-1; //we have no more space to store anything

   pthread_rwlock_rdlock(&hw->movemutex);
   if (hw->x_external) hw->scan_x_data[hw->scan_ndata] = (hw->in[hw->x_external_source] + hw->x_external_offset)*hw->x_external_slope; 
   else hw->scan_x_data[hw->scan_ndata] = hw->xpos;

   if (hw->y_external) hw->scan_y_data[hw->scan_ndata] = (hw->in[hw->y_external_source] + hw->y_external_offset)*hw->y_external_slope; 
   else hw->scan_y_data[hw->scan_ndata] = hw->ypos;
   pthread_rwlock_unlock(&hw->movemutex);

   pthread_rwlock_rdlock(&hw->cardmutex);
   if (hw->subtract_slope) hw->scan_z_data[hw->scan_ndata] = hw->zpiezo - (librp_GetPidOffset(&hw->rp) - hw->zshift)/hw->zcal;
   else hw->scan_z_data[hw->scan_ndata] = generate_fake_z_point(hw->xpos, hw->ypos, hw->xrange, hw->yrange);
   hw->scan_e_data[hw->scan_ndata] = hw->errorsignal;
   hw->scan_ts_data[hw->scan_ndata] = hw->timestamp;
   if (hw->c_sc_a1) hw->scan_a1_data[hw->scan_ndata] = hw->amplitude1;
   if (hw->c_sc_p1) hw->scan_p1_data[hw->scan_ndata] = hw->phase1;
   if (hw->c_sc_a2) hw->scan_a2_data[hw->scan_ndata] = hw->amplitude2;
   if (hw->c_sc_p2) hw->scan_p2_data[hw->scan_ndata] = hw->phase2;
   if (hw->c_sc_kpfm) hw->scan_kpfm_data[hw->scan_ndata] = hw->f2_offset;
   if (hw->c_sc_dart) hw->scan_dart_data[hw->scan_ndata] = hw->dart_frequency + hw->dart_frequencyshift;
   if (hw->c_sc_l1x) hw->scan_l1x_data[hw->scan_ndata] = hw->l1x;
   if (hw->c_sc_l1y) hw->scan_l1y_data[hw->scan_ndata] = hw->l1y;
   if (hw->c_sc_l2x) hw->scan_l2x_data[hw->scan_ndata] = hw->l2x;
   if (hw->c_sc_l2y) hw->scan_l2y_data[hw->scan_ndata] = hw->l2y;
   if (hw->c_sc_out9) hw->scan_out9_data[hw->scan_ndata] = hw->aout[8];
   if (hw->c_sc_slp) hw->scan_slp_data[hw->scan_ndata] = (librp_GetPidOffset(&hw->rp) - hw->zshift)/hw->zcal;


   for (i=0; i<16; i++) if (hw->c_sc_in[i]) hw->scan_in_data[i][hw->scan_ndata] = hw->in[i];

   if (hw->c_sc_set) hw->scan_s_data[hw->scan_ndata] = set;
   if (hw->c_sc_fmd) hw->scan_fmd_data[hw->scan_ndata] = hw->amresult;
   pthread_rwlock_unlock(&hw->cardmutex);

//   printf("stored %d: %g %g %g\n", hw->scan_ndata, hw->scan_x_data[hw->scan_ndata], hw->scan_y_data[hw->scan_ndata], hw->scan_z_data[hw->scan_ndata]);

   hw->scan_ndata++;

   return hw->scan_ndata-1;

}
