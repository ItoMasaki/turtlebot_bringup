#include <stdint.h>

#include "libkobuki.h"
#include "kobukicwrapper.h"

#define KOBUKI_OK 0
#define INVALID_ARG -1

using namespace rt_net;

#define MAX_KOBUKI 16
static uint32_t kobuki_count;
static Kobuki* g_kobuki[MAX_KOBUKI];


#define KOBUKI_TEST(handle) do {if((handle) < 0 || (handle) >= (int32_t)kobuki_count || g_kobuki[handle] == NULL) return INVALID_ARG;} while(false)

LIBKOBUKI_API H_KOBUKI Kobuki_create(const char* arg)
{
  KobukiStringArgument strArg(arg);
  try {
    g_kobuki[kobuki_count] = createKobuki(strArg);
  } catch (...) {
    return -1;
  }
  kobuki_count++;
  return kobuki_count -1;
}


LIBKOBUKI_API RETVAL Kobuki_destroy(H_KOBUKI kobuki) {
  KOBUKI_TEST(kobuki);

  delete g_kobuki[kobuki];
  g_kobuki[kobuki] = NULL;

  return KOBUKI_OK;
}


LIBKOBUKI_API RETVAL Kobuki_setTargetVelocity(H_KOBUKI kobuki, double vx, double va)
{
  try {
    KOBUKI_TEST(kobuki);
    g_kobuki[kobuki]->setTargetVelocity(vx, va);
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

LIBKOBUKI_API RETVAL Kobuki_setDigitalOut(H_KOBUKI kobuki,const uint8_t channel, const bool flag) {
  try { KOBUKI_TEST(kobuki);
    g_kobuki[kobuki]->setDigitalOut((GPIO)channel, flag);
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}


/**
 * @brief 電源ピン
 * @param channel チャンネル
 * @param flag trueなら出力ON
 */
LIBKOBUKI_API RETVAL Kobuki_setExternalPower(H_KOBUKI kobuki,const uint8_t channel, const bool flag) {
  try { KOBUKI_TEST(kobuki);
    g_kobuki[kobuki]->setExternalPower((POWER)channel, flag);
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

    
/**
 * @brief LED1の設定
 * @param color 色(H_KOBUKI kobuki,GREEN/RED)
 * @param flag trueならON
 *
 */
LIBKOBUKI_API RETVAL Kobuki_setLED1(H_KOBUKI kobuki,const uint8_t color, const bool flag) {
  try { KOBUKI_TEST(kobuki);
    g_kobuki[kobuki]->setLED1((COLOR)color, flag);
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}


/**
 * @brief LED2の設定
 * @param color 色(GREEN/RED)
 * @param flag trueならON
 */
LIBKOBUKI_API RETVAL Kobuki_setLED2(H_KOBUKI kobuki,const uint8_t color, const bool flag) {
  try { KOBUKI_TEST(kobuki);
    g_kobuki[kobuki]->setLED2((COLOR)color, flag);
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}


/**
 * @brief ボタン入力の検出
 * @return B0が押されていたらtrue
 */
LIBKOBUKI_API RETVAL Kobuki_isButton0(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isButton0();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}


/**
 * @brief ボタン入力の検出
 * @return B1が押されていたらtrue
 */
LIBKOBUKI_API RETVAL Kobuki_isButton1(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isButton1();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}


/**
 * @brief ボタン入力の検出
 * @return B2が押されていたらtrue
 */
LIBKOBUKI_API RETVAL Kobuki_isButton2(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isButton2();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}


/**
 * @brief 右バンパーの接触
 * @return 接触したらtrueを返す
 */
LIBKOBUKI_API RETVAL Kobuki_isRightBump(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isRightBump();

  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

    
/**
 * @brief 中央部バンパーの接触
 * @return 接触したらtrueを返す
 */
LIBKOBUKI_API RETVAL Kobuki_isCenterBump(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isCenterBump();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

    
/**
 * @brief 左バンパーの接触
 * @return 接触したらtrueを返す
 */
LIBKOBUKI_API RETVAL Kobuki_isLeftBump(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isLeftBump();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

    
/**
 * @brief 右車輪の落下
 * @return 落下したらtrueを返す
     */
LIBKOBUKI_API RETVAL Kobuki_isRightWheelDrop(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isRightWheelDrop();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

    
/**
 * @brief 左車輪の落下
 * @return 落下したらtrueを返す
 */
LIBKOBUKI_API RETVAL Kobuki_isLeftWheelDrop(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isLeftWheelDrop();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

    
/**
 * @brief 右側下方向の赤外線センサの反応
 * @return センサ反応がなくなればtrue （崖であると判定）
 */
LIBKOBUKI_API RETVAL Kobuki_isRightCliff(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isRightCliff();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}


/**
 * @brief 左側下方向の赤外線センサの反応
 * @return センサ反応がなくなればtrue （崖であると判定）
 */
LIBKOBUKI_API RETVAL Kobuki_isLeftCliff(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isLeftCliff();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
 * @brief 中央下方向の赤外線センサの反応
 * @return センサ反応がなくなればtrue （崖であると判定）
 */
LIBKOBUKI_API RETVAL Kobuki_isCenterCliff(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isCenterCliff();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
 * @brief ドックセンサからの赤外線反応
 * @return 
 */
LIBKOBUKI_API RETVAL Kobuki_isRightIRFarRight(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isRightIRFarRight();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
 * @brief ドックセンサからの赤外線反応
 * @return 
 */
LIBKOBUKI_API RETVAL Kobuki_isRightIRFarCenter(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isRightIRFarCenter();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
 * @brief ドックセンサからの赤外線反応
 * @return 
 */
LIBKOBUKI_API RETVAL Kobuki_isRightIRFarLeft(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isRightIRFarLeft();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
 * @brief ドックセンサからの赤外線反応
 * @return 
 */
LIBKOBUKI_API RETVAL Kobuki_isRightIRNearRight(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isRightIRNearRight();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
 * @brief ドックセンサからの赤外線反応
 * @return 
 */
LIBKOBUKI_API RETVAL Kobuki_isRightIRNearCenter(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isRightIRNearCenter();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
 * @brief ドックセンサからの赤外線反応
 * @return 
 */
LIBKOBUKI_API RETVAL Kobuki_isRightIRNearLeft(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isRightIRNearLeft();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
 * @brief ドックセンサからの赤外線反応
 * @return 
 */
LIBKOBUKI_API RETVAL Kobuki_isCenterIRFarRight(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isCenterIRFarRight();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}


/**
 * @brief ドックセンサからの赤外線反応
 * @return 
 */
LIBKOBUKI_API RETVAL Kobuki_isCenterIRFarCenter(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isCenterIRFarCenter();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}


/**
 * @brief ドックセンサからの赤外線反応
 * @return 
 */
LIBKOBUKI_API RETVAL Kobuki_isCenterIRFarLeft(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isCenterIRFarLeft();    
    
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
 * @brief ドックセンサからの赤外線反応
 * @return 
 */
LIBKOBUKI_API RETVAL Kobuki_isCenterIRNearRight(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isCenterIRNearRight();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
 * @brief ドックセンサからの赤外線反応
 * @return 
 */
LIBKOBUKI_API RETVAL Kobuki_isCenterIRNearCenter(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isCenterIRNearCenter();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
 * @brief ドックセンサからの赤外線反応
 * @return 
 */
LIBKOBUKI_API RETVAL Kobuki_isCenterIRNearLeft(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isCenterIRNearLeft();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}


/**
 * @brief ドックセンサからの赤外線反応
 * @return 
 */
LIBKOBUKI_API RETVAL Kobuki_isLeftIRFarRight(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isLeftIRFarRight();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
 * @brief ドックセンサからの赤外線反応
 * @return 
 */
LIBKOBUKI_API RETVAL Kobuki_isLeftIRFarCenter(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isLeftIRFarCenter();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
 * @brief ドックセンサからの赤外線反応
 * @return 
 */
LIBKOBUKI_API RETVAL Kobuki_isLeftIRFarLeft(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isLeftIRFarLeft();    
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
 * @brief ドックセンサからの赤外線反応
 * @return 
 */
LIBKOBUKI_API RETVAL Kobuki_isLeftIRNearRight(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isLeftIRNearRight();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
 * @brief ドックセンサからの赤外線反応
 * @return 
 */
LIBKOBUKI_API RETVAL Kobuki_isLeftIRNearCenter(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isLeftIRNearCenter();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
 * @brief ドックセンサからの赤外線反応
 * @return 
 */
LIBKOBUKI_API RETVAL Kobuki_isLeftIRNearLeft(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isLeftIRNearLeft();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
 * @brief ドックに接触しているか
 * @return ドックに入って入ればtrue
 */
LIBKOBUKI_API RETVAL Kobuki_isDocked(H_KOBUKI kobuki, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->isDocked();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
 * @brief バッテリ電圧取得
 * @return バッテリ電圧（単位V）
 */
LIBKOBUKI_API RETVAL Kobuki_getBatteryVoltage(H_KOBUKI kobuki, double* voltage) {
  try { KOBUKI_TEST(kobuki);
    *voltage = g_kobuki[kobuki]->getBatteryVoltage();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}


/**
 * @brief オドメトリからの現在位置推定
 * @return 現在のX軸方向の移動距離（m）
 */
LIBKOBUKI_API RETVAL Kobuki_getPoseX(H_KOBUKI kobuki, double* meter) {
  try { KOBUKI_TEST(kobuki);
    *meter = g_kobuki[kobuki]->getPoseX();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}


/**
 * @brief オドメトリからの現在位置推定
 * @return 現在のY軸方向の移動距離（m）
 */
LIBKOBUKI_API RETVAL Kobuki_getPoseY(H_KOBUKI kobuki, double* meter) {
  try { KOBUKI_TEST(kobuki);
    *meter = g_kobuki[kobuki]->getPoseY();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}


/**
 * @brief オドメトリからの現在位置推定
 * @return 現在の向いている方向（rad）
 */
LIBKOBUKI_API RETVAL Kobuki_getPoseTh(H_KOBUKI kobuki, double* radian) {
  try { KOBUKI_TEST(kobuki);
    *radian = g_kobuki[kobuki]->getPoseTh();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
 * @brief オドメトリからの現在位置推定値の強制更新
 * @param x x軸方向(m)
 * @param y y軸方向(m)
 * @param th 向いている方向(rad)
 */
LIBKOBUKI_API RETVAL Kobuki_setPose(H_KOBUKI kobuki, double x, double y, double th) 
{
  try { KOBUKI_TEST(kobuki);
    g_kobuki[kobuki]->setPose(x, y, th);
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
* @brief オドメトリからの現在位置推定値の同時取得
* @param x x軸方向(m)
* @param y y軸方向(m)
* @param th 向いている方向(rad)
*/
LIBKOBUKI_API RETVAL Kobuki_getPose(H_KOBUKI kobuki, double* x, double* y, double* th)
{
  try { KOBUKI_TEST(kobuki);
    g_kobuki[kobuki]->getPose(x, y, th);
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
 * @brief 右モータの電流値
 * @return 電流値(H_KOBUKI kobuki,A)
 */
LIBKOBUKI_API RETVAL Kobuki_getRightMotorCurrent(H_KOBUKI kobuki, double* current) {
  try { KOBUKI_TEST(kobuki);
    *current = g_kobuki[kobuki]->getRightMotorCurrent();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}


/**
 * @brief 左モータの電流値
 * @return 電流値(H_KOBUKI kobuki,A)
 */
LIBKOBUKI_API RETVAL Kobuki_getLeftMotorCurrent(H_KOBUKI kobuki, double* current) {
  try { KOBUKI_TEST(kobuki);
    *current = g_kobuki[kobuki]->getLeftMotorCurrent();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}

/**
 * @brief デジタル入力
 * @param ピン番号(H_KOBUKI kobuki,0-4)
 * @return trueならHIGH
 */
LIBKOBUKI_API RETVAL Kobuki_getDigitalIn(H_KOBUKI kobuki,const uint8_t ch, bool* flag) {
  try { KOBUKI_TEST(kobuki);
    *flag = g_kobuki[kobuki]->getDigitalIn(ch);
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}


/**
 * @brief アナログ入力
 * @param ピン番号(0-4)
 * @return 電圧(0-3.3V)
 */
LIBKOBUKI_API RETVAL Kobuki_getAnalogIn(H_KOBUKI kobuki,const uint8_t ch, double* voltage) {
  try { KOBUKI_TEST(kobuki);
    *voltage = g_kobuki[kobuki]->getAnalogIn(ch);
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}


/**
 * @brief ドックに向かう
 * @return 
 */
LIBKOBUKI_API RETVAL Kobuki_dock(H_KOBUKI kobuki,const bool block, int32_t *retval){
  try { KOBUKI_TEST(kobuki);
    *retval = g_kobuki[kobuki]->dock(block);
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}


/**
 * @brief ドック状態を取得
 * @return 
 */
LIBKOBUKI_API RETVAL Kobuki_getDockState(H_KOBUKI kobuki,const bool block, int32_t *retval){
  try { KOBUKI_TEST(kobuki);
    *retval = g_kobuki[kobuki]->getDockState();
  } catch (...) {
    return INVALID_ARG;
  }
  return KOBUKI_OK;
}


/**
 * @brief でバッグ用関数 echo
 */
LIBKOBUKI_API uint32_t Kobuki_echo(uint32_t arg) {
  return arg;
}

