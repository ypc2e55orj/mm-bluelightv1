#include "config.h"

#include <cJSON.h>
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
  RunParameterConfig paramFast[FAST_PARAMETER_COUNTS] = {
    {
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
    }
  };

  void setPIDConfig(cJSON *object, PIDConfig &config)
  {
    cJSON *kp = cJSON_GetObjectItem(object, "kp");
    cJSON *ki = cJSON_GetObjectItem(object, "ki");
    cJSON *kd = cJSON_GetObjectItem(object, "kd");

    if (cJSON_IsNumber(kp))
    {
      config.kp = static_cast<float>(kp->valuedouble);
    }
    if (cJSON_IsNumber(ki))
    {
      config.ki = static_cast<float>(ki->valuedouble);
    }
    if (cJSON_IsNumber(kd))
    {
      config.kd = static_cast<float>(kd->valuedouble);
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
      config.maxAngVelo = static_cast<float>(maxAngVelo->valuedouble);
    }
    if (cJSON_IsNumber(maxAngAccel))
    {
      config.maxAngAccel = static_cast<float>(maxAngAccel->valuedouble);
    }
    if (cJSON_IsNumber(maxAngJerk))
    {
      config.maxAngJerk = static_cast<float>(maxAngJerk->valuedouble);
    }

    if (pid)
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
      config.offsetPre = static_cast<float>(offsetPre->valuedouble);
    }
    if (cJSON_IsNumber(offsetPost))
    {
      config.offsetPost = static_cast<float>(offsetPost->valuedouble);
    }

    if (turn)
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
      config.maxVelo = static_cast<float>(maxVelo->valuedouble);
    }
    if (cJSON_IsNumber(maxAccel))
    {
      config.maxAccel = static_cast<float>(maxAccel->valuedouble);
    }
    if (cJSON_IsNumber(maxJerk))
    {
      config.maxJerk = static_cast<float>(maxJerk->valuedouble);
    }

    if (pid)
    {
      setPIDConfig(pid, config.pid);
    }
  }
  void setRunParameterConfig(cJSON *object, RunParameterConfig &config)
  {
    cJSON *straight = cJSON_GetObjectItem(object, "straight");
    cJSON *turn = cJSON_GetObjectItem(object, "turn");
    cJSON *slalomTurn = cJSON_GetObjectItem(object, "SlalomTurn");

    if (straight)
    {
      setStraightConfig(straight, config.straight);
    }
    if (turn)
    {
      setTurnConfig(turn, config.turn);
    }
    if (slalomTurn)
    {
      setSlalomTurnConfig(slalomTurn, config.slalomTurn);
    }
  }
  void setParamFast(cJSON *object, RunParameterConfig config[FAST_PARAMETER_COUNTS])
  {
    for (int i = 0; i < FAST_PARAMETER_COUNTS; i++)
    {
      cJSON *param = cJSON_GetArrayItem(object, i);
      if (param)
      {
        setRunParameterConfig(param, config[i]);
      }
    }
  }
  void setConfigPair(cJSON *object, ConfigPair &config)
  {
    cJSON *left = cJSON_GetObjectItem(object, "left");
    cJSON *right = cJSON_GetObjectItem(object, "right");

    if (cJSON_IsNumber(left))
    {
      config.left = static_cast<float>(left->valuedouble);
    }
    if (cJSON_IsNumber(right))
    {
      config.right = static_cast<float>(right->valuedouble);
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
      config.vehicleWeight = static_cast<float>(vehicleWeight->valuedouble);
    }
    if (cJSON_IsNumber(vehicleTread))
    {
      config.vehicleTread = static_cast<float>(vehicleTread->valuedouble);
    }
    if (cJSON_IsNumber(vehicleInertiaMoment))
    {
      config.vehicleInertiaMoment = static_cast<float>(vehicleInertiaMoment->valuedouble);
    }
    if (cJSON_IsNumber(tireDiameter))
    {
      config.tireDiameter = static_cast<float>(tireDiameter->valuedouble);
    }
    if (cJSON_IsNumber(spurGearTeeth))
    {
      config.spurGearTeeth = spurGearTeeth->valueint;
    }
    if (cJSON_IsNumber(pinionGearTeeth))
    {
      config.pinionGearTeeth = pinionGearTeeth->valueint;
    }
    if (motorResistance)
    {
      setConfigPair(motorResistance, config.motorResistance);
    }
    if (motorInductance)
    {
      setConfigPair(motorInductance, config.motorInductance);
    }
    if (motorBackForce)
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

    if (goal)
    {
      setConfigCoord(goal, config.goal);
    }
    if (size)
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
    free(string);

    if (!object)
    {
      assert(false);
    }
    cJSON *mazeConfig = cJSON_GetObjectItem(object, "maze");
    if (mazeConfig)
    {
      setMazeConfig(mazeConfig, maze);
    }
    cJSON *hardwareConfig = cJSON_GetObjectItem(object, "hardware");
    if (hardwareConfig)
    {
      setHardwareConfig(hardwareConfig, hardware);
    }
    cJSON *paramSearchConfig = cJSON_GetObjectItem(object, "paramSearch");
    if (paramSearchConfig)
    {
      setRunParameterConfig(paramSearchConfig, paramSearch);
    }
    cJSON *paramFastConfig = cJSON_GetObjectItem(object, "paramFast");
    if (paramFastConfig)
    {
      setParamFast(paramFastConfig, paramFast);
    }

    cJSON_Delete(object);
  }

  cJSON *getPIDConfig(const PIDConfig &config)
  {
    cJSON *pid = cJSON_CreateObject();
    if (!pid)
    {
      return nullptr;
    }

    cJSON_AddNumberToObject(pid, "kp", config.kp);
    cJSON_AddNumberToObject(pid, "ki", config.ki);
    cJSON_AddNumberToObject(pid, "kd", config.kd);

    return pid;
  }
  cJSON *getTurnConfig(TurnConfig &config)
  {
    cJSON *turn = cJSON_CreateObject();
    cJSON *pid = getPIDConfig(config.pid);
    if (!turn)
    {
      goto err;
    }
    if (!pid)
    {
      goto err;
    }

    cJSON_AddNumberToObject(turn, "maxAngVelo", config.maxAngVelo);
    cJSON_AddNumberToObject(turn, "maxAngAccel", config.maxAngAccel);
    cJSON_AddNumberToObject(turn, "maxAngJerk", config.maxAngJerk);
    cJSON_AddItemToObject(turn, "pid", pid);

    return turn;

  err:
    cJSON_Delete(turn);
    cJSON_Delete(pid);

    return nullptr;
  }
  cJSON *getSlalomTurnConfig(SlalomTurnConfig &config)
  {
    cJSON *slalomTurn = cJSON_CreateObject();
    cJSON *turn = getTurnConfig(config.turn);
    if (!slalomTurn)
    {
      goto err;
    }
    if (!turn)
    {
      goto err;
    }

    cJSON_AddItemToObject(slalomTurn, "turn", turn);
    cJSON_AddNumberToObject(slalomTurn, "offsetPre", config.offsetPre);
    cJSON_AddNumberToObject(slalomTurn, "offsetPost", config.offsetPost);

    return slalomTurn;

  err:
    cJSON_Delete(slalomTurn);
    cJSON_Delete(turn);

    return nullptr;
  }
  cJSON *getStraightConfig(StraightRunConfig &config)
  {
    cJSON *straight = cJSON_CreateObject();
    cJSON *pid = getPIDConfig(config.pid);
    if (!straight)
    {
      goto err;
    }
    if (!pid)
    {
      goto err;
    }

    cJSON_AddNumberToObject(straight, "maxVelo", config.maxVelo);
    cJSON_AddNumberToObject(straight, "maxAccel",config.maxAccel);
    cJSON_AddNumberToObject(straight, "maxJerk", config.maxJerk);
    cJSON_AddItemToObject(straight, "pid", pid);

    return straight;

  err:
    cJSON_Delete(straight);
    cJSON_Delete(pid);

    return nullptr;
  }
  cJSON *getRunParameterConfig(RunParameterConfig &config)
  {
    cJSON *runParameter = cJSON_CreateObject();
    cJSON *straight = getStraightConfig(config.straight);
    cJSON *turn = getTurnConfig(config.turn);
    cJSON *slalomTurn = getSlalomTurnConfig(config.slalomTurn);
    if (!runParameter)
    {
      goto err;
    }
    if (!straight)
    {
      goto err;
    }
    if (!turn)
    {
      goto err;
    }
    if (!slalomTurn)
    {
      goto err;
    }

    cJSON_AddItemToObject(runParameter, "straight", straight);
    cJSON_AddItemToObject(runParameter, "turn", turn);
    cJSON_AddItemToObject(runParameter, "slalomTurn", slalomTurn);

    return runParameter;

  err:
    cJSON_Delete(runParameter);
    cJSON_Delete(straight);
    cJSON_Delete(turn);
    cJSON_Delete(slalomTurn);

    return nullptr;
  }
  cJSON *getConfigPair(ConfigPair &config)
  {
    cJSON *configPair = cJSON_CreateObject();
    if (!configPair)
    {
      return nullptr;
    }

    cJSON_AddNumberToObject(configPair, "left", config.left);
    cJSON_AddNumberToObject(configPair, "right", config.right);

    return configPair;
  }
  cJSON *getHardwareConfig(HardwareConfig &config)
  {
    cJSON *hardwareConfig = cJSON_CreateObject();
    cJSON *motorResistance = getConfigPair(config.motorResistance);
    cJSON *motorInductance = getConfigPair(config.motorInductance);
    cJSON *motorBackForce = getConfigPair(config.motorBackForce);
    if (!hardwareConfig)
    {
      goto err;
    }
    if (!motorResistance)
    {
      goto err;
    }
    if (!motorInductance)
    {
      goto err;
    }
    if (!motorBackForce)
    {
      goto err;
    }

    cJSON_AddNumberToObject(hardwareConfig, "vehicleWeight", config.vehicleWeight);
    cJSON_AddNumberToObject(hardwareConfig, "vehicleTread", config.vehicleTread);
    cJSON_AddNumberToObject(hardwareConfig, "vehicleInertiaMoment", config.vehicleInertiaMoment);
    cJSON_AddNumberToObject(hardwareConfig, "tireDiameter", config.tireDiameter);
    cJSON_AddNumberToObject(hardwareConfig, "spurGearTeeth", config.spurGearTeeth);
    cJSON_AddNumberToObject(hardwareConfig, "pinionGearTeeth", config.pinionGearTeeth);
    cJSON_AddItemToObject(hardwareConfig, "motorResistance", motorResistance);
    cJSON_AddItemToObject(hardwareConfig, "motorInductance", motorInductance);
    cJSON_AddItemToObject(hardwareConfig, "motorBackForce", motorBackForce);

    return hardwareConfig;

  err:
    cJSON_Delete(hardwareConfig);
    cJSON_Delete(motorResistance);
    cJSON_Delete(motorInductance);
    cJSON_Delete(motorBackForce);

    return nullptr;
  }
  cJSON *getConfigCoord(ConfigCoord &config)
  {
    cJSON *configCoord = cJSON_CreateObject();
    if (!configCoord)
    {
      return nullptr;
    }

    cJSON_AddNumberToObject(configCoord, "x", config.x);
    cJSON_AddNumberToObject(configCoord, "y", config.y);

    return configCoord;
  }
  cJSON *getMazeConfig(MazeConfig &config)
  {
    cJSON *mazeConfig = cJSON_CreateObject();
    cJSON *goal = getConfigCoord(config.goal);
    cJSON *size = getConfigCoord(config.size);
    if (!mazeConfig)
    {
      goto err;
    }
    if (!goal)
    {
      goto err;
    }
    if (!size)
    {
      goto err;
    }

    cJSON_AddItemToObject(mazeConfig, "goal", goal);
    cJSON_AddItemToObject(mazeConfig, "size", size);

    return mazeConfig;

  err:
    cJSON_Delete(mazeConfig);
    cJSON_Delete(goal);
    cJSON_Delete(size);

    return nullptr;
  }

  void write(const char *path)
  {
    cJSON *config = cJSON_CreateObject();
    if (!config)
    {
      assert(false);
    }
    cJSON *mazeConfig = getMazeConfig(maze);
    if (!mazeConfig)
    {
      assert(false);
    }
    cJSON *hardwareConfig = getHardwareConfig(hardware);
    if (!hardwareConfig)
    {
      assert(false);
    }
    cJSON *paramSearchConfig = getRunParameterConfig(paramSearch);
    if (!paramSearchConfig)
    {
      assert(false);
    }
    cJSON *paramFastConfig = cJSON_CreateArray();
    if (!paramFastConfig)
    {
      assert(false);
    }
    for (auto & param : paramFast)
    {
      cJSON *fast = getRunParameterConfig(param);
      if (!fast)
      {
        assert(false);
      }
      cJSON_AddItemToArray(paramFastConfig, fast);
    }

    cJSON_AddItemToObject(config, "maze", mazeConfig);
    cJSON_AddItemToObject(config, "hardware", hardwareConfig);
    cJSON_AddItemToObject(config, "paramSearch", paramSearchConfig);
    cJSON_AddItemToObject(config, "paramFast", paramFastConfig);

    char *string = cJSON_Print(config);
    cJSON_Delete(config);

    FILE *fp = fopen(path, "w");
    fprintf(fp, "%s\n", string);
    fclose(fp);

    delete string;
  }
}
