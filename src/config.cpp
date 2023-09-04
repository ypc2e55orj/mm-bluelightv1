#include "config.h"

#include <cJSON.h>
#include <cstddef>
#include <cstdio>
#include <cstdlib>

namespace config
{
  MazeConfig maze = {
    .goal = {15, 15}, // 0 ~
    .size = {32, 32},
  };
  HardwareConfig hardware = {
    .vehicleWeight = 10.0f, // [g]
    .vehicleTread = 3.80f, // [mm]
    .vehicleInertiaMoment = 1.0f, // []
    .tireDiameter = 12.80f, // [mm]
    .spurGearTeeth = 38, // []
    .pinionGearTeeth = 9, // []
    .motorResistance = {
      .left = 5.063f, // [ohm]
      .right = 5.063f, // [ohm]
    },
    .motorInductance = {
      .left = 29.1f, // [uH]
      .right = 29.1f, // [uH]
    },
    .motorBackForce = {
      .left = 1.0f, // []
      .right = 1.0f, // []
    }
  };
  RunParameterConfig paramSearch = {
    .straight = {
      .maxVelo = 0.3f,  // [m/s]
      .maxAccel = 1.5f, // [m/s^2]
      .maxJerk = 1000.0f, // [m/s^3]
      .pid = { 10.0f, 0.05f, 0.0f },
    },
    .turn = {
      .maxAngVelo = 10.0f, // [rad/s]
      .maxAngAccel = 20.0f, // [rad/s^2]
      .maxAngJerk = 1000.0f, // [rad/s^3]
      .pid = { 0.6f, 0.01f, 0.0f },
    },
    .slalomTurn = {
      .turn = {
        .maxAngVelo = 10.0f, // [rad/s]
        .maxAngAccel = 40.0f, // [rad/s^2]
        .maxAngJerk = 1000.0f, // [rad/s^3]
        .pid = { 0.6f, 0.01f, 0.0f },
      },
      .offsetPre = 16.0f, // [mm]
      .offsetPost = 13.0f, // [mm]
    }
  };
  RunParameterConfig paramFast[5] = {};

  void setPIDConfig(cJSON *object, PIDConfig &config)
  {
    cJSON *kp = cJSON_GetObjectItem(object, "kp");
    cJSON *ki = cJSON_GetObjectItem(object, "ki");
    cJSON *kd = cJSON_GetObjectItem(object, "kd");

    if (cJSON_IsNumber(kp))
    {
      config.kp = kp->valuedouble;
    }
    if (cJSON_IsNumber(ki))
    {
      config.ki = ki->valuedouble;
    }
    if (cJSON_IsNumber(kd))
    {
      config.kd = kd->valuedouble;
    }
  }
  void setTurnConfig(cJSON *object, TurnConfig &config)
  {
    cJSON *maxAngVelo = cJSON_GetObjectItem(object, "maxAngVelo");
    cJSON *maxAngAccel = cJSON_GetObjectItem(object, "maxAngAccel");
    cJSON *maxAngJerk = cJSON_GetObjectItem(object, "maxAngJerk");
    cJSON *pid = cJSON_GetObjectItem(object, "pid");

    if (cJSON_IsNumber(maxAngVelo))
    {
      config.maxAngVelo = maxAngVelo->valuedouble;
    }
    if (cJSON_IsNumber(maxAngAccel))
    {
      config.maxAngAccel = maxAngAccel->valuedouble;
    }
    if (cJSON_IsNumber(maxAngJerk))
    {
      config.maxAngJerk = maxAngJerk->valuedouble;
    }

    if (pid != nullptr)
    {
      setPIDConfig(pid, config.pid);
    }
  }
  void setSlalomTurnConfig(cJSON *object, SlalomTurnConfig &config)
  {
    cJSON *offsetPre = cJSON_GetObjectItem(object, "offsetPre");
    cJSON *offsetPost = cJSON_GetObjectItem(object, "offsetPost");
    cJSON *turn = cJSON_GetObjectItem(object, "turn");

    if (cJSON_IsNumber(offsetPre))
    {
      config.offsetPre = offsetPre->valuedouble;
    }
    if (cJSON_IsNumber(offsetPost))
    {
      config.offsetPost = offsetPost->valuedouble;
    }

    if (turn != nullptr)
    {
      setTurnConfig(turn, config.turn);
    }
  }
  void setStraightConfig(cJSON *object, StraightRunConfig &config)
  {
    cJSON *maxVelo = cJSON_GetObjectItem(object, "maxVelo");
    cJSON *maxAccel = cJSON_GetObjectItem(object, "maxAccel");
    cJSON *maxJerk = cJSON_GetObjectItem(object, "maxJerk");
    cJSON *pid = cJSON_GetObjectItem(object, "pid");

    if (cJSON_IsNumber(maxVelo))
    {
      config.maxVelo = maxVelo->valuedouble;
    }
    if (cJSON_IsNumber(maxAccel))
    {
      config.maxAccel = maxAccel->valuedouble;
    }
    if (cJSON_IsNumber(maxJerk))
    {
      config.maxJerk = maxJerk->valuedouble;
    }

    if (pid != nullptr)
    {
      setPIDConfig(pid, config.pid);
    }
  }
  void setRunParameterConfig(cJSON *object, RunParameterConfig &config)
  {
    cJSON *straight = cJSON_GetObjectItem(object, "straight");
    cJSON *turn = cJSON_GetObjectItem(object, "turn");
    cJSON *slalomTurn = cJSON_GetObjectItem(object, "SlalomTurn");


    setStraightConfig(straight, config.straight);
    setTurnConfig(object, config.turn);
    setSlalomTurnConfig(object, config.slalomTurn);
  }
  void setConfigPair(cJSON *object, ConfigPair &config)
  {
    cJSON *left = cJSON_GetObjectItem(object, "left");
    cJSON *right = cJSON_GetObjectItem(object, "right");

    if (cJSON_IsNumber(left))
    {
      config.left = left->valuedouble;
    }
    if (cJSON_IsNumber(right))
    {
      config.right = right->valuedouble;
    }
  }
  void setHardwareConfig(cJSON *object, HardwareConfig &config)
  {
    cJSON *vehicleWeight = cJSON_GetObjectItem(object, "vehicleWeight");
    cJSON *vehicleTread = cJSON_GetObjectItem(object, "vehicleTread");
    cJSON *vehicleInertiaMoment = cJSON_GetObjectItem(object, "vehicleInertiaMoment");
    cJSON *tireDiameter = cJSON_GetObjectItem(object, "tireDiameter");
    cJSON *spurGearTeeth = cJSON_GetObjectItem(object, "spurGearTeeth");
    cJSON *pinionGearTeeth = cJSON_GetObjectItem(object, "pinionGearTeeth");
    cJSON *motorResistance = cJSON_GetObjectItem(object, "motorResistance");
    cJSON *motorInductance = cJSON_GetObjectItem(object, "motorInductance");
    cJSON *motorBackForce = cJSON_GetObjectItem(object, "motorBackForce");

    if (cJSON_IsNumber(vehicleWeight))
    {
      config.vehicleWeight = vehicleWeight->valuedouble;
    }
    if (cJSON_IsNumber(vehicleTread))
    {
      config.vehicleTread = vehicleTread->valuedouble;
    }
    if (cJSON_IsNumber(vehicleInertiaMoment))
    {
      config.vehicleInertiaMoment = vehicleInertiaMoment->valuedouble;
    }
    if (cJSON_IsNumber(tireDiameter))
    {
      config.tireDiameter = tireDiameter->valuedouble;
    }
    if (cJSON_IsNumber(spurGearTeeth))
    {
      config.spurGearTeeth = spurGearTeeth->valueint;
    }
    if (cJSON_IsNumber(pinionGearTeeth))
    {
      config.pinionGearTeeth = pinionGearTeeth->valueint;
    }
    if (motorResistance != nullptr)
    {
      setConfigPair(motorResistance, config.motorResistance);
    }
    if (motorInductance != nullptr)
    {
      setConfigPair(motorInductance, config.motorInductance);
    }
    if (motorBackForce != nullptr)
    {
      setConfigPair(motorBackForce, config.motorBackForce);
    }
  }
  void setConfigCoord(cJSON *object, ConfigCoord &config)
  {
    cJSON *x = cJSON_GetObjectItem(object, "x");
    cJSON *y = cJSON_GetObjectItem(object, "y");

    if (cJSON_IsNumber(x))
    {
      config.x = x->valueint;
    }
    if (cJSON_IsNumber(y))
    {
      config.y = y->valueint;
    }
  }
  void setMazeConfig(cJSON *object, MazeConfig &config)
  {
    cJSON *goal = cJSON_GetObjectItem(object, "goal");
    cJSON *size = cJSON_GetObjectItem(object, "size");

    if (goal != nullptr)
    {
      setConfigCoord(goal, config.goal);
    }
    if (size != nullptr)
    {
      setConfigCoord(size, config.size);
    }
  }

  void read(const char *path)
  {
    FILE *fp = fopen(path, "r");
    if (!fp)
    {
      return;
    }

    fseek(fp, 0, SEEK_END);
    size_t size = ftell(fp);
    rewind(fp);
    char *string = (char *)malloc((size + 1) * sizeof(char));
    fread(string, 1, size, fp);
    string[size] = '\0';
    fclose(fp);

    cJSON *object = cJSON_Parse(string);
    setMazeConfig(cJSON_GetObjectItem(object, "maze"), maze);
    setHardwareConfig(cJSON_GetObjectItem(object, "hardware"), hardware);
    setRunParameterConfig(cJSON_GetObjectItem(object, "paramSearch"), paramSearch);
    for (int i = 0; i < FAST_PARAMETER_COUNTS; i++)
    {
      cJSON *fast = cJSON_GetArrayItem(cJSON_GetObjectItem(object, "paramFast"), i);
      setRunParameterConfig(fast, paramFast[i]);
    }
    free(string);
    cJSON_Delete(object);
  }

  cJSON *getPIDConfig(PIDConfig &config)
  {
    cJSON *pid = cJSON_CreateObject();

    cJSON_AddNumberToObject(pid, "kp", config.kp);
    cJSON_AddNumberToObject(pid, "ki", config.ki);
    cJSON_AddNumberToObject(pid, "kd", config.kd);

    return pid;
  }
  cJSON *getTurnConfig(TurnConfig &config)
  {
    cJSON *turn = cJSON_CreateObject();

    cJSON_AddNumberToObject(turn, "maxAngVelo", config.maxAngVelo);
    cJSON_AddNumberToObject(turn, "maxAngAccel", config.maxAngAccel);
    cJSON_AddNumberToObject(turn, "maxAngJerk", config.maxAngJerk);

    cJSON_AddItemToObject(turn, "pid", getPIDConfig(config.pid));

    return turn;
  }
  cJSON *getSlalomTurnConfig(SlalomTurnConfig &config)
  {
    cJSON *slalomTurn = cJSON_CreateObject();

    cJSON_AddItemToObject(slalomTurn, "turn", getTurnConfig(config.turn));
    cJSON_AddNumberToObject(slalomTurn, "offsetPre", config.offsetPre);
    cJSON_AddNumberToObject(slalomTurn, "offsetPost", config.offsetPost);

    return slalomTurn;
  }
  cJSON *getStraightConfig(StraightRunConfig &config)
  {
    cJSON *straight = cJSON_CreateObject();

    cJSON_AddNumberToObject(straight, "maxVelo", config.maxVelo);
    cJSON_AddNumberToObject(straight, "maxAccel",config.maxAccel);
    cJSON_AddNumberToObject(straight, "maxJerk", config.maxJerk);

    cJSON_AddItemToObject(straight, "pid", getPIDConfig(config.pid));

    return straight;
  }
  cJSON *getRunParameterConfig(RunParameterConfig &config)
  {
    cJSON *runParameter = cJSON_CreateObject();

    cJSON_AddItemToObject(runParameter, "straight", getStraightConfig(config.straight));
    cJSON_AddItemToObject(runParameter, "turn", getTurnConfig(config.turn));
    cJSON_AddItemToObject(runParameter, "slalomTurn", getSlalomTurnConfig(config.slalomTurn));

    return runParameter;
  }
  cJSON *getConfigPair(ConfigPair &config)
  {
    cJSON *configPair = cJSON_CreateObject();
    cJSON_AddNumberToObject(configPair, "left", config.left);
    cJSON_AddNumberToObject(configPair, "right", config.right);
    return configPair;
  }
  cJSON *getHardwareConfig(HardwareConfig &config)
  {
    cJSON *hardware = cJSON_CreateObject();

    cJSON_AddNumberToObject(hardware, "vehicleWeight", config.vehicleWeight);
    cJSON_AddNumberToObject(hardware, "vehicleTread", config.vehicleTread);
    cJSON_AddNumberToObject(hardware, "vehicleInertiaMoment", config.vehicleInertiaMoment);
    cJSON_AddNumberToObject(hardware, "tireDiameter", config.tireDiameter);
    cJSON_AddNumberToObject(hardware, "spurGearTeeth", config.spurGearTeeth);
    cJSON_AddNumberToObject(hardware, "pinionGearTeeth", config.pinionGearTeeth);
    cJSON_AddItemToObject(hardware, "motorResistance", getConfigPair(config.motorResistance));
    cJSON_AddItemToObject(hardware, "motorInductance", getConfigPair(config.motorInductance));
    cJSON_AddItemToObject(hardware, "motorBackForce", getConfigPair(config.motorBackForce));

    return hardware;
  }
  cJSON *getConfigCoord(ConfigCoord &config)
  {
    cJSON *configCoord = cJSON_CreateObject();

    cJSON_AddNumberToObject(configCoord, "x", config.x);
    cJSON_AddNumberToObject(configCoord, "y", config.y);

    return configCoord;
  }
  cJSON *getMazeConfig(MazeConfig &config)
  {
    cJSON *maze = cJSON_CreateObject();

    cJSON_AddItemToObject(maze, "goal", getConfigCoord(config.goal));
    cJSON_AddItemToObject(maze, "size", getConfigCoord(config.size));

    return maze;
  }

  void write(const char *path)
  {
    cJSON *config = cJSON_CreateObject();
    cJSON_AddItemToObject(config, "maze", getMazeConfig(maze));
    cJSON_AddItemToObject(config, "hardware", getHardwareConfig(hardware));
    cJSON_AddItemToObject(config, "paramSearch", getRunParameterConfig(paramSearch));
    cJSON *fast = cJSON_CreateArray();
    for (int i = 0; i < FAST_PARAMETER_COUNTS; i++)
    {
      cJSON_AddItemToArray(fast, getRunParameterConfig(paramFast[i]));
    }
    cJSON_AddItemToObject(config, "paramFast", fast);

    char *string = cJSON_Print(config);
    cJSON_Delete(config);

    FILE *fp = fopen(path, "w");
    fprintf(fp, "%s\n", string);
    fclose(fp);

    delete string;
  }
}
