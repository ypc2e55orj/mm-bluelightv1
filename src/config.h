#pragma once

#include <cstdint>

namespace config
{
  struct ConfigPair
  {
    float left;
    float right;
  };
  struct HardwareConfig
  {
    float vehicleWeight;
    float vehicleTread;
    float vehicleInertiaMoment;
    float tireDiameter;
    int spurGearTeeth;
    int pinionGearTeeth;
    ConfigPair motorResistance;
    ConfigPair motorInductance;
    ConfigPair motorBackForce;
  };
  struct PIDConfig
  {
    float kp;
    float ki;
    float kd;
  };
  struct StraightRunConfig
  {
    float maxVelo;
    float maxAccel;
    float maxJerk;
    PIDConfig pid;
  };
  struct TurnConfig
  {
    float maxAngVelo;
    float maxAngAccel;
    float maxAngJerk;
    PIDConfig pid;
  };
  struct SlalomTurnConfig
  {
    TurnConfig turn;
    float offsetPre;
    float offsetPost;
  };
  struct RunParameterConfig
  {
    StraightRunConfig straight;
    TurnConfig turn;
    SlalomTurnConfig slalomTurn;
  };
  struct ConfigCoord
  {
    int x;
    int y;
  };
  struct MazeConfig
  {
    ConfigCoord goal;
    ConfigCoord size;
  };

  void read(const char *path);
  void write(const char *path);

  constexpr int FAST_PARAMETER_COUNTS = 5;
  extern MazeConfig maze;
  extern HardwareConfig hardware;
  extern RunParameterConfig paramSearch;
  extern RunParameterConfig paramFast[FAST_PARAMETER_COUNTS];
}
