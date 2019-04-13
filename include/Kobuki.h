#pragma once
#include <stdint.h>
#include <string>
#include <iostream>


#include "libkobuki.h"

#ifndef LIBKOBUKI_API

#ifdef WIN32
#ifdef _WINDLL
#define LIBKOBUKI_API __declspec(dllexport)
#else // _WINDLL
#define LIBKOBUKI_API __declspec(dllimport)
#endif // _WINDLL
#else // WIN32
#define LIBKOBUKI_API 
#endif // ifdef WIN32

#endif //LIBKOBUKI_API

/**
 * @brief RTcorp用ネームスペース
 */
namespace rt_net {

  /**
   * @brief Kobukiクラス用引数インターフェース
   */
  class KobukiArgument {
    
    /**
     * @brief KobukiArgumentの文字列表現化
     */
    virtual const std::string toString() const = 0;
  };

  /**
   * @brief KobukiStringArgumentクラス
   */
  class KobukiStringArgument : public KobukiArgument {
  private:
    std::string m_str;

  public:
    /**
     * @brief コンストラクタ．
     * 
     * @param arg Kobukiクラスに渡す文字列の引数
     */
    KobukiStringArgument(const std::string& arg) {
      m_str = arg;
    }
    
    /**
     * @brief コンストラクタ．
     * 
     * @param str Kobukiクラスに渡す文字列の引数
     */
    KobukiStringArgument(const char* str) {
      m_str = str;
    }

    
    /**
     * @brief デストラクタ
     */
    virtual ~KobukiStringArgument() {}

    /**
     * @brief KobukiArgumentの文字列表現化
     */
    virtual const std::string toString() const {
      return m_str;
    }
  };


    
  /**
   * @brief Kobukiインターフェースクラス
   */
  class LIBKOBUKI_API Kobuki {
  public:
    /**
     * @brief コンストラクタ
     * このコンストラクタは使えません．createKobuki()関数を利用してください．
     *
     */
    Kobuki(){}
    
    /**
     * @brief デストラクタ
     */
    virtual ~Kobuki(){}
    
    
  public:
  
    /**
     * @brief 目標速度の設定
     *
     * @param vx 直進方向（X軸）の速度 (m/sec)
     * @param va 回転方向（Z軸）の回転速度（rad/sec）
     */
    virtual void setTargetVelocity(const double& vx, const double& va) = 0;
    
    /**
     * @brief ディジタル出力ピンの設定
     * @param channel チャンネル
     * @param flag trueならHIGH，falseならLOW
     */
    virtual void setDigitalOut(const GPIO channel, const bool flag) = 0;

    /**
     * @brief 電源ピン
     * @param channel チャンネル
     * @param flag trueなら出力ON
     */
    virtual void setExternalPower(const POWER channel, const bool flag) = 0;
    
    /**
     * @brief LED1の設定
     * @param color 色(GREEN/RED)
     * @param flag trueならON
     *
     */
    virtual void setLED1(const COLOR color, const bool flag) = 0;

    /**
     * @brief LED2の設定
     * @param color 色(GREEN/RED)
     * @param flag trueならON
     */
    virtual void setLED2(const COLOR color, const bool flag) = 0;

    /**
     * @brief ボタン入力の検出
     * @return B0が押されていたらtrue
     */
    virtual bool isButton0() = 0;

    /**
     * @brief ボタン入力の検出
     * @return B1が押されていたらtrue
     */
    virtual bool isButton1() = 0;

    /**
     * @brief ボタン入力の検出
     * @return B2が押されていたらtrue
     */
    virtual bool isButton2() = 0;

    /**
     * @brief 右バンパーの接触
     * @return 接触したらtrueを返す
     */
    virtual bool isRightBump() = 0;
    
    /**
     * @brief 中央部バンパーの接触
     * @return 接触したらtrueを返す
     */
    virtual bool isCenterBump() = 0;
    
    /**
     * @brief 左バンパーの接触
     * @return 接触したらtrueを返す
     */
    virtual bool isLeftBump() = 0;
    
    /**
     * @brief 右車輪の落下
     * @return 落下したらtrueを返す
     */
    virtual bool isRightWheelDrop() = 0;
    
    /**
     * @brief 左車輪の落下
     * @return 落下したらtrueを返す
     */
    virtual bool isLeftWheelDrop() = 0;
    
    /**
     * @brief 右側下方向の赤外線センサの反応
     * @return センサ反応がなくなればtrue （崖であると判定）
     */
    virtual bool isRightCliff() = 0;

    /**
     * @brief 左側下方向の赤外線センサの反応
     * @return センサ反応がなくなればtrue （崖であると判定）
     */
    virtual bool isLeftCliff() = 0;

    /**
     * @brief 中央下方向の赤外線センサの反応
     * @return センサ反応がなくなればtrue （崖であると判定）
     */
    virtual bool isCenterCliff() = 0;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return ロボット右側のIRセンサに，ドッキングステーションから見て右側遠方領域反応ならtrue
     */
    virtual bool isRightIRFarRight() = 0;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return ロボット右側のIRセンサに，ドッキングステーションから見て中央遠方領域反応ならtrue
     */
    virtual bool isRightIRFarCenter() = 0;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return ロボット右側のIRセンサに，ドッキングステーションから見て左側遠方領域反応ならtrue
     */
    virtual bool isRightIRFarLeft() = 0;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return ロボット右側のIRセンサに，ドッキングステーションから見て右側近接領域反応ならtrue
     */
    virtual bool isRightIRNearRight() = 0;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return ロボット右側のIRセンサに，ドッキングステーションから見て中央近接領域反応ならtrue
     */
    virtual bool isRightIRNearCenter() = 0;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return ロボット右側のIRセンサに，ドッキングステーションから見て左側近接領域反応ならtrue
     */
    virtual bool isRightIRNearLeft() = 0;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return ロボット中央のIRセンサに，ドッキングステーションから見て右側遠方領域反応ならtrue
     */
    virtual bool isCenterIRFarRight() = 0;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return ロボット中央のIRセンサに，ドッキングステーションから見て中央遠方領域反応ならtrue
     */
    virtual bool isCenterIRFarCenter() = 0;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return ロボット中央のIRセンサに，ドッキングステーションから見て左側遠方領域反応ならtrue
     */
    virtual bool isCenterIRFarLeft() = 0;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return ロボット中央のIRセンサに，ドッキングステーションから見て右側近接領域反応ならtrue
     */
    virtual bool isCenterIRNearRight() = 0;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return ロボット中央のIRセンサに，ドッキングステーションから見て中央近接領域反応ならtrue
     */
    virtual bool isCenterIRNearCenter() = 0;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return ロボット中央のIRセンサに，ドッキングステーションから見て左側近接領域反応ならtrue
     */
    virtual bool isCenterIRNearLeft() = 0;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return ロボット左側のIRセンサに，ドッキングステーションから見て右側遠方領域反応ならtrue
     */
    virtual bool isLeftIRFarRight() = 0;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return ロボット左側のIRセンサに，ドッキングステーションから見て中央遠方領域反応ならtrue
     */
    virtual bool isLeftIRFarCenter() = 0;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return ロボット左側のIRセンサに，ドッキングステーションから見て左側遠方領域反応ならtrue
     */
    virtual bool isLeftIRFarLeft() = 0;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return ロボット左側のIRセンサに，ドッキングステーションから見て右側遠方領域反応ならtrue
     */
    virtual bool isLeftIRNearRight() = 0;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return ロボット左側のIRセンサに，ドッキングステーションから見て中央遠方領域反応ならtrue
     */
    virtual bool isLeftIRNearCenter() = 0;

    /**
     * @brief ドックセンサからの赤外線反応
     * @return ロボット左側のIRセンサに，ドッキングステーションから見て左側遠方領域反応ならtrue
     */
    virtual bool isLeftIRNearLeft() = 0;


    /**
     * @brief 充電中か否か
     * @return 充電中ならtrue．充電完了もしくは，放電中ならfalse
     */
    virtual bool isCharging() = 0;


    /**
     * @brief 充電中か否か？
     * @return 充電完了ならtrue．充電中もしくは，放電中ならばfalse
     */
    virtual bool isCharged() = 0;

    /**
     * @brief 放電中か否か？
     * @return 放電中ならばtrue．充電完了もしくは，充電中ならばfalse
     */
    virtual bool isDischarging() = 0;
    
    /**
     * @brief ドックに接触しているか
     * @return ドックに入って入ればtrue
     */
    virtual bool isDocked() = 0;

    /**
     * @brief バッテリ電圧取得
     * @return バッテリ電圧（単位V）
     */
    virtual double getBatteryVoltage() = 0;

    /**
     * @brief オドメトリからの現在位置推定
     * @return 現在のX軸方向の移動距離（m）
     */
    virtual double getPoseX() = 0;

    /**
     * @brief オドメトリからの現在位置推定
     * @return 現在のY軸方向の移動距離（m）
     */
    virtual double getPoseY() = 0;

    /**
     * @brief オドメトリからの現在位置推定
     * @return 現在の向いている方向（rad）
     */
    virtual double getPoseTh() = 0;

    /**
     * @brief オドメトリからの現在位置推定値の同時取得
     * @param x x軸方向(m)
     * @param y y軸方向(m)
     * @param th 向いている方向(rad)
     */
	virtual void getPose(double* x, double* y, double* th) = 0;

    /**
     * @brief オドメトリからの現在位置推定値の強制更新
     * @param x x軸方向(m)
     * @param y y軸方向(m)
     * @param th 向いている方向(rad)
     */
    virtual void setPose(double x, double y, double th) = 0;
    
    /**
     * @brief 右モータの電流値
     * @return 電流値(A)
     */
    virtual double getRightMotorCurrent() = 0;

    /**
     * @brief 左モータの電流値
     * @return 電流値(A)
     */
    virtual double getLeftMotorCurrent() = 0;


    /**
     * @brief 右車輪のエンコーダ入力
     * @return エンコーダの生データ
     */
    virtual uint16_t getRightMotorEncoder() = 0;

    /**
     * @brief 左車輪のエンコーダ入力
     * @return エンコーダの生データ
     */
    virtual uint16_t getLeftMotorEncoder() = 0;


    /**
     * @brief デジタル入力
     * @param ch ピン番号(0-4)
     * @return trueならHIGH
     */
    virtual bool getDigitalIn(const uint8_t ch) = 0;

    /**
     * @brief アナログ入力
     * @param ch ピン番号(0-4)
     * @return 電圧(0-3.3V)
     */
    virtual double getAnalogIn(const uint8_t ch) = 0;

    /**
     * @brief ドックに向かう
     * @return ドッキング状態．blockがfalseのときは DOCKING ．blockがtrueのときは，ドッキング動作が終了した場合の状態が返る．
     */
    virtual DOCKSTATE dock(const bool block = true) = 0;

	/**
	 * @brief ドッキング動作の状態取得
	 * @return ドッキング状態．充電状態を知りたければ， isChargingなどを参照すること．
	 */
	virtual DOCKSTATE getDockState() = 0;

  };

}
