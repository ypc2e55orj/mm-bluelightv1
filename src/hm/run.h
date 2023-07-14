#pragma once

void straight(float len, float acc, float max_sp, float end_sp);
void straight_debug(float len, float acc, float max_sp, float end_sp);

void turn(int deg, float ang_acc, float max_om, short dir);
void turn_debug(int deg, float ang_acc, float max_om, short dir);

void slalom_turn(int deg, float ang_acc, float max_om, short dir, float end_sp);

void adjust_fwall(void);
void adjust_turn(void);
void start_position(void);
