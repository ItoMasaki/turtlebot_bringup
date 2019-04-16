/** @file libkobuki.h
 * @mail staff@rt-net.jp
 * @url http://rt-net.jp
 * @copyright RT Corp. 2013 All rights reserved.
 */

/**
 * @mainpage
 * 
 * @section libkobuki 汎用ロボット台車Kobuki用ライブラリ
 * 
 * libkobukiはYujin Robotics社製汎用ロボット台車「Kobuki」を利用するための標準ライブラリです．
 * libkobukiは標準でC++から利用することがでます．
 * 本ライブラリは，Linux, MacOSX, Windows7で利用することができます．
 *
 * @section 概要
 * C++においては，リファレンスマニュアルの rt_net::Kobuki クラスを参照してください．
 * rt_net::Kobuki クラスは汎用台車用APIを実装したインターフェースです．
 *
 * @section Kobukiクラスオブジェクトの生成
 * rt_net::Kobuki クラスのオブジェクトを生成する場合は， createKobuki( const rt_net::KobukiArgument& ) グローバル関数を呼び出します．
 * createKobuki( ) 関数は rt_net::Kobuki オブジェクトを生成するFactory関数です．
 * createKobuki( ) 関数の引数には，利用するCOMポートを示す文字列が必要になります．Linuxなどでは，/dev/ttyUSB0などのデバイスファイル名です．
 * @code
 * 
 * #include "libkobuki.h"
 * 
 * using namespace rt_net;
 *
 * int main(void) {
 *   Kobuki* kobuki = createKobuki(KobukiStringArgument("COM3"));
 *   ... 何か処理 ...
 *   delete kobuki;
 *   return 0;
 * }
 * @endcode
 * Windowsの場合，createKobuki( ) 関数の引数のCOMに続く番号が9を超える場合は注意が必要です．その場合はデバイスマネージャから，COM以下の番号を9以下に変更するか，下記のコードのように，エスケープ文字列を追加する必要があります．
 * @code
 * 
 * #include "libkobuki.h"
 * 
 * using namespace rt_net;
 *
 * int main(void) {
 *   Kobuki* kobuki = createKobuki(KobukiStringArgument("\\\\.\\COM10"));
 *   ....なにか処理....
 *   delete kobuki;
 *   return 0;
 * }
 * @endcode
 *
 * @section Kobukiへの指令
 *
 * Kobuki のオブジェクトのメンバ関数を使えば，簡単にKobuki本体を動作させることができます．代表的なものをお見せします．
 * 
 * @subsection 速度指令
 * 速度指令には，rt_net::Kobuki::setTargetVelocity(double, double) 関数を使います．
 * @code
 * 
 * #include "libkobuki.h"
 * 
 * using namespace rt_net;
 *
 * int main(void) {
 *   Kobuki* kobuki = createKobuki(KobukiStringArgument("\\\\.\\COM10"));
 *   
 *   kobuki->setTargetVelocity(0.05, 0.0); // 前方に50mm/secで移動．
 *
 *   (void)getchar(); // Enterが押されるまで待つ
 *
 *   delete kobuki;
 *   return 0;
 * }
 * @endcode
 * 
 * @subsection スイッチ等の取得
 * 接触センサがKobukiの前方に搭載されています．接触は，左右および正面の三か所を区別することができます．
 * @code
 * 
 * #include "libkobuki.h"
 * 
 * using namespace rt_net;
 *
 * int main(void) {
 *   Kobuki* kobuki = createKobuki(KobukiStringArgument("\\\\.\\COM10"));
 *   
 *   if(kobuki->isRightBump()) {
 *     printf("Right Bumper Touched.\n");
 *   }
 *
 *   (void)getchar(); // Enterが押されるまで待つ
 *
 *   delete kobuki;
 *   return 0;
 * }
 * @endcode
 * また，Kobuki後部のボタン（B0, B1, B2）の入力を判定するisButton0()関数や，タイヤの「浮き」を検出するisRightWheelDrop(), isLeftWheelDrop()関数などが用意されています．
 *
 * @subsection ドッキングステーションへの期間
 * rt_net::Kobuki::dock( ) 関数が用意されています．
 * 
 * @section その他
 * libkobukiをインストールすると，exampleフォルダが生成されます．Windowsでは通常，C:\Program Files (x86)\RT_net\libkobuki内にあります．
 * Linuxでは，/usr/local/shareの中です．
 * サンプルコードをビルドするには，CMake2.8が必要です．CMakeの使い方は，CMakeのマニュアルを参照してください．
 * また，Windowsにおいては，CMakeプロセス内で，環境変数LIBKOBUKI_ROOTを使います．バイナリインストーラでは，これを自動的に設定しますが，Windowsの再起動が必要になります．
 * ソースコードからビルドする場合は，LIBKOBUKI_ROOTを手動で設定する必要があります．通常は，C:\Program Files (x86)\RT_net\libkobukiなどになります．
 * CMakeを使わず，Visual Studioなどから，直接，includeフォルダのヘッダーをインクルードし，libフォルダ内のkobuki.libをリンクすることも可能です．
 * 
 * @section トラブルシューティング
 * 一部のバーチャルマシン環境では，Kobukiのドライバが正常に動作しない場合が報告されていますので，動作の保障はしかねます．
 * 
 *
 * @example demo.cpp
 */
 
#pragma once

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
* @brief GPIO Channel Number
*/
enum GPIO {
	GPIO_CH0,
	GPIO_CH1,
	GPIO_CH2,
	GPIO_CH3
};

/**
* @brief Voltage Supply Channel Number 
*/
enum POWER {
	POWER_33V,
	POWER_50V,
	POWER_12V1A,
	POWER_12V5A,
};

/**
* @brief LED Color
*/
enum COLOR {
	RED,
	GREEN,
};

enum DOCKSTATE {
	IDLE = 0,
	DOCKED = 1,
	DOCKING = 2,
	BUMPSTOP = -1,
	CLIFFSTOP = -2,
	DROPSTOP = -3,
	LOSTSTOP = -4,
};

#ifdef __cplusplus
#include "Kobuki.h"


namespace rt_net {
	class Kobuki;
	class KobukiArgument;
}

/**
 * @brief Kobukiインターフェースのオブジェクト作成
 *
 * @param arg KobukiArgumentインターフェースを実装したオブジェクトを入れる．（現バージョンではKobukiStringArgument）
 * @return rt_net::Kobuki クラスオブジェクトへのポインタ
 */
LIBKOBUKI_API rt_net::Kobuki* createKobuki(const rt_net::KobukiArgument &arg);

#endif

#include "kobukicwrapper.h"
