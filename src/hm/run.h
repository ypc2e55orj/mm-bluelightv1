#pragma once

void straight(float len, float acc, float max_sp, float end_sp);
void turn(int deg, float ang_acc, float max_om, short dir);

void slalom_turn(int deg, float ang_acc, float max_om, short dir, float end_sp);

void check_straight(float end_sp);
void get_adjust_len(float * r_len, float * l_len);

void adjust_fwall(void);
void adjust_turn(void);
void start_position(void);
