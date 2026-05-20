#include "Vision.h"
#include "HUSKYLENS.h"

HUSKYLENS huskylens;

int xCenter = 0;
int yCenter = 0;
int width = 0;
int height = 0;
bool isTagVisable = false;

int get_xCenter(){ return xCenter;}
int get_yCenter(){ return yCenter;}
int get_width(){ return width;}
int get_height(){ return height;}
bool get_isTagVisable(){ return isTagVisable;}

void beginVision(){
  if (!huskylens.begin(Wire)) {
    Serial.println(F("HuskyLens begin failed!"));
  }

  huskylens.writeAlgorithm(ALGORITHM_TAG_RECOGNITION);
}


void updateVision() {
  if (huskylens.requestBlocks() && huskylens.countBlocks() >= 1){
    isTagVisable = true;

    HUSKYLENSResult result = huskylens.getBlock(0); // get tag with ID zero
    xCenter = result.xCenter;
    yCenter = result.yCenter;
    width = result.width ;
    height = result.height;
  } else {
    isTagVisable = false;
  }
}

void printVision() {
  Serial.println(String()+ F("xCenter= ") + xCenter + F(" ,yCenter= ") + yCenter + F(" ,width= ") + width + F(" ,height= ") + height);
}

// 1. The physical width of the black square of your April Tag in mm
#define REAL_TAG_WIDTH_MM  41.0  

// 2. The resolution width of your camera (e.g., 320, 640, 800)
#define CAMERA_RES_X       320.0
#define CAMERA_RES_Y       240.0  

// 3. The value you calculate in the Calibration step
#define MEASUREMENT_DISTANCE_MM 300.0
#define MEASUREMENT_WIDTH_PIXLES 44.0


// --- VARIABLES TO STORE RESULTS ---
float distanceVision = 0;
float xOffsetVision = 0;

// --- THE HELPER FUNCTION ---
void calculatePositionVision(float tagX, float tagWidth) {
  float focal_length = MEASUREMENT_WIDTH_PIXLES * MEASUREMENT_DISTANCE_MM / REAL_TAG_WIDTH_MM;

  // Prevent divide by zero errors if the camera loses the tag
  if (!isTagVisable) {
    return;
  }

  // 1. Calculate Distance (Z-axis)
  // Distance = (Real Width * Focal Length) / Pixel Width
  distanceVision = (REAL_TAG_WIDTH_MM * focal_length) / tagWidth;

  // 2. Calculate Offset (X-axis)
  // First, find how far the tag is from the center of the screen in pixels
  float screenCenter = CAMERA_RES_X / 2.0;
  float pixelOffset = tagX - screenCenter;
  // Convert that pixel offset to mm using the geometry of similar triangles
  // Formula: Offset_mm = (PixelOffset * Distance) / FocalLength
  xOffsetVision = (pixelOffset * distanceVision) / focal_length;
  
  //Serial.printf("%f %f %f %f %f %f \r\n", focal_length, tagX, tagWidth, distanceVision, pixelOffset, xOffsetVision );
}

float get_distanceVision(){
  return distanceVision;
}

float get_xOffsetVision(){
  return xOffsetVision;
}