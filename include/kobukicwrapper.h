/**







 **/


#pragma once 


#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>

  typedef int32_t H_KOBUKI;
  typedef int32_t RETVAL;

  LIBKOBUKI_API H_KOBUKI Kobuki_create(const char* arg);
  
  LIBKOBUKI_API RETVAL Kobuki_destroy(H_KOBUKI kobuki);

    /**
     * @brief 目標速度の設定
     *
     * @param vx 直進方向（X軸）の速度 (H_KOBUKI kobuki,m/sec)
     * @param va 回転方向（Z軸）の回転速度（rad/sec）
     */
    LIBKOBUKI_API RETVAL Kobuki_setTargetVelocity(H_KOBUKI kobuki, double vx, double va) ;
    
    /**
     * @brief ディジタル出力ピンの設定
     * @param channel チャンネル
     * @param flag trueならHIGH，falseならLOW
     */
    LIBKOBUKI_API RETVAL Kobuki_setDigitalOut(H_KOBUKI kobuki,const uint8_t channel, const bool flag) ;

    /**
     * @brief 電源ピン
     * @param channel チャンネル
     * @param flag trueなら出力ON
     */
    LIBKOBUKI_API RETVAL Kobuki_setExternalPower(H_KOBUKI kobuki,const uint8_t channel, const bool flag) ;
    
    /**
     * @brief LED1の設定
     * @param color 色(H_KOBUKI kobuki,GREEN/RED)
     * @param flag trueならON
     *
     */
    LIBKOBUKI_API RETVAL Kobuki_setLED1(H_KOBUKI kobuki,const uint8_t color, const bool flag) ;

    /**
     * @brief LED2の設定
     * @param color 色(GREEN/RED)
     * @param flag trueならON
     */
    LIBKOBUKI_API RETVAL Kobuki_setLED2(H_KOBUKI kobuki,const uint8_t color, const bool flag) ;

    /**
     * @brief ボタン入力の検出
     * @return B0が押されていたらtrue
     */
    LIBKOBUKI_API RETVAL Kobuki_isButton0(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief ボタン入力の検出
     * @return B1が押されていたらtrue
     */
    LIBKOBUKI_API RETVAL Kobuki_isButton1(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief ボタン入力の検出
     * @return B2が押されていたらtrue
     */
    LIBKOBUKI_API RETVAL Kobuki_isButton2(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief 右バンパーの接触
     * @return 接触したらtrueを返す
     */
    LIBKOBUKI_API RETVAL Kobuki_isRightBump(H_KOBUKI kobuki, bool* flag) ;
    
    /**
     * @brief 中央部バンパーの接触
     * @return 接触したらtrueを返す
     */
    LIBKOBUKI_API RETVAL Kobuki_isCenterBump(H_KOBUKI kobuki, bool* flag) ;
    
    /**
     * @brief 左バンパーの接触
     * @return 接触したらtrueを返す
     */
    LIBKOBUKI_API RETVAL Kobuki_isLeftBump(H_KOBUKI kobuki, bool* flag) ;
    
    /**
     * @brief 右車輪の落下
     * @return 落下したらtrueを返す
     */
    LIBKOBUKI_API RETVAL Kobuki_isRightWheelDrop(H_KOBUKI kobuki, bool* flag) ;
    
    /**
     * @brief 左車輪の落下
     * @return 落下したらtrueを返す
     */
    LIBKOBUKI_API RETVAL Kobuki_isLeftWheelDrop(H_KOBUKI kobuki, bool* flag) ;
    
    /**
     * @brief 右側下方向の赤外線センサの反応
     * @return センサ反応がなくなればtrue （崖であると判定）
     */
    LIBKOBUKI_API RETVAL Kobuki_isRightCliff(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief 左側下方向の赤外線センサの反応
     * @return センサ反応がなくなればtrue （崖であると判定）
     */
    LIBKOBUKI_API RETVAL Kobuki_isLeftCliff(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief 中央下方向の赤外線センサの反応
     * @return センサ反応がなくなればtrue （崖であると判定）
     */
    LIBKOBUKI_API RETVAL Kobuki_isCenterCliff(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return 
     */
    LIBKOBUKI_API RETVAL Kobuki_isRightIRFarRight(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return 
     */
    LIBKOBUKI_API RETVAL Kobuki_isRightIRFarCenter(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return 
     */
    LIBKOBUKI_API RETVAL Kobuki_isRightIRFarLeft(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return 
     */
    LIBKOBUKI_API RETVAL Kobuki_isRightIRNearRight(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return 
     */
    LIBKOBUKI_API RETVAL Kobuki_isRightIRNearCenter(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return 
     */
    LIBKOBUKI_API RETVAL Kobuki_isRightIRNearLeft(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return 
     */
    LIBKOBUKI_API RETVAL Kobuki_isCenterIRFarRight(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return 
     */
    LIBKOBUKI_API RETVAL Kobuki_isCenterIRFarCenter(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return 
     */
    LIBKOBUKI_API RETVAL Kobuki_isCenterIRFarLeft(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return 
     */
    LIBKOBUKI_API RETVAL Kobuki_isCenterIRNearRight(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return 
     */
    LIBKOBUKI_API RETVAL Kobuki_isCenterIRNearCenter(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return 
     */
    LIBKOBUKI_API RETVAL Kobuki_isCenterIRNearLeft(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return 
     */
    LIBKOBUKI_API RETVAL Kobuki_isLeftIRFarRight(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return 
     */
    LIBKOBUKI_API RETVAL Kobuki_isLeftIRFarCenter(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return 
     */
    LIBKOBUKI_API RETVAL Kobuki_isLeftIRFarLeft(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return 
     */
    LIBKOBUKI_API RETVAL Kobuki_isLeftIRNearRight(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return 
     */
    LIBKOBUKI_API RETVAL Kobuki_isLeftIRNearCenter(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return 
     */
    LIBKOBUKI_API RETVAL Kobuki_isLeftIRNearLeft(H_KOBUKI kobuki, bool* flag) ;



    /**
     * @brief 充電中か否か
     * @return 充電中ならtrue．充電完了もしくは，放電中ならfalse
     */
    LIBKOBUKI_API RETVAL Kobuki_isCharging(H_KOBUKI kobuki, bool* flag);


    /**
     * @brief 充電中か否か？
     * @return 充電完了ならtrue．充電中もしくは，放電中ならばfalse
     */
    LIBKOBUKI_API RETVAL Kobuki_isCharged(H_KOBUKI kobuki, bool* flag);

    /**
     * @brief 放電中か否か？
     * @return 放電中ならばtrue．充電完了もしくは，充電中ならばfalse
     */
    LIBKOBUKI_API RETVAL Kobuki_isDischarging(H_KOBUKI kobuki, bool* flag);
   
    
    /**
     * @brief ドックに接触しているか
     * @return ドックに入って入ればtrue
     */
    LIBKOBUKI_API RETVAL Kobuki_isDocked(H_KOBUKI kobuki, bool* flag) ;

    /**
     * @brief バッテリ電圧取得
     * @return バッテリ電圧（単位V）
     */
  LIBKOBUKI_API RETVAL Kobuki_getBatteryVoltage(H_KOBUKI kobuki, double* voltage) ;

    /**
     * @brief オドメトリからの現在位置推定
     * @return 現在のX軸方向の移動距離（m）
     */
  LIBKOBUKI_API RETVAL Kobuki_getPoseX(H_KOBUKI kobuki, double* meter) ;

    /**
     * @brief オドメトリからの現在位置推定
     * @return 現在のY軸方向の移動距離（m）
     */
  LIBKOBUKI_API RETVAL Kobuki_getPoseY(H_KOBUKI kobuki, double* meter) ;

    /**
     * @brief オドメトリからの現在位置推定
     * @return 現在の向いている方向（rad）
     */
  LIBKOBUKI_API RETVAL Kobuki_getPoseTh(H_KOBUKI kobuki, double* radian) ;

    /**
     * @brief オドメトリからの現在位置推定値の強制更新
     * @param x x軸方向(m)
     * @param y y軸方向(m)
     * @param th 向いている方向(rad)
     */
  LIBKOBUKI_API RETVAL Kobuki_setPose(H_KOBUKI kobuki, double x, double y, double th) ;

    /**
     * @brief オドメトリからの現在位置推定値の同時取得
     * @param x x軸方向(m)
     * @param y y軸方向(m)
     * @param th 向いている方向(rad)
     */
  LIBKOBUKI_API RETVAL Kobuki_getPose(H_KOBUKI kobuki, double* x, double* y, double* th);

    /**
     * @brief 右モータの電流値
     * @return 電流値(H_KOBUKI kobuki,A)
     */
  LIBKOBUKI_API RETVAL Kobuki_getRightMotorCurrent(H_KOBUKI kobuki, double* current) ;

    /**
     * @brief 左モータの電流値
     * @return 電流値(H_KOBUKI kobuki,A)
     */
  LIBKOBUKI_API RETVAL Kobuki_getLeftMotorCurrent(H_KOBUKI kobuki, double* current) ;

    /**
     * @brief デジタル入力
     * @param ピン番号(H_KOBUKI kobuki,0-4)
     * @return trueならHIGH
     */
    LIBKOBUKI_API RETVAL Kobuki_getDigitalIn(H_KOBUKI kobuki,const uint8_t ch) ;

    /**
     * @brief アナログ入力
     * @param ピン番号(0-4)
     * @return 電圧(0-3.3V)
     */
  LIBKOBUKI_API RETVAL Kobuki_getAnalogIn(H_KOBUKI kobuki,const uint8_t ch, double* voltage) ;



    /**
     * @brief ドックに向かう
     * @return 
     */
  LIBKOBUKI_API RETVAL Kobuki_dock(H_KOBUKI kobuki, const bool block, int32_t *retval);

  

	/**
	 * @brief ドック状態を取得
	 * @return 
	 */
	LIBKOBUKI_API RETVAL Kobuki_getDockState(H_KOBUKI kobuki,const bool block, int32_t *retval);

  LIBKOBUKI_API uint32_t Kobuki_echo(uint32_t arg);

#ifdef __cplusplus
}
#endif


