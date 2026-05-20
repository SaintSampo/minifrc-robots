#ifndef VISION_H
#define VISION_H

int get_xCenter();
int get_yCenter();
int get_width();
int get_height();
bool get_isTagVisable();

void beginVision();
void updateVision();
void printVision();

void calculatePositionVision(float tagX, float tagWidth);
float get_distanceVision();
float get_xOffsetVision();

#endif