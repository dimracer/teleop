#!/usr/bin/env python3
# coding=utf-8
"""
calibrate_approach.py

Помогает подобрать APPROACH для pult_pickup_teleop.py: подключается к PiPER
ТОЛЬКО НА ЧТЕНИЕ (не включает CAN-режим, не шлёт команд движения) и в
реальном времени печатает текущие углы суставов, пока ты вручную двигаешь
руку в режиме drag-teach. Когда рука встанет в нужную позу над предметом —
жмёшь Ctrl+C, скрипт печатает готовую строку для вставки в pult_pickup_teleop.py.

ПОРЯДОК ДЕЙСТВИЙ (делай именно в этом порядке):

  1. Подъезжаешь на роботе примерно туда, откуда обычно будешь подъезжать
     к предмету (типичное расстояние/угол).
  2. На самой руке жмёшь кнопку между J5 и J6 ОДИН раз -> загорается ровный
     зелёный свет -> рука входит в drag-teach (её можно свободно двигать
     руками, моторы поддаются).
  3. Запускаешь этот скрипт:
         python3 calibrate_approach.py --piper-can can_piper
  4. Руками аккуратно подводишь захват в позу "approach" -- это НЕ поза
     самого захвата предмета, а точка чуть в стороне/выше, откуда потом
     джойстиком (правый стик + VRA) будет вестись точная донастройка.
     Оставляй запас 5-10 см от стола/предмета -- не нужно вплотную.
  5. Смотришь на экран -- углы суставов обновляются в реальном времени.
     Как только поза устраивает -- жмёшь Ctrl+C (можно одной рукой,
     вторая держит руку в этот момент, чтобы не сместилась).
  6. Скрипт напечатает готовую строку вида:
         APPROACH = JointAngles(j1=.., j2=.., j3=.., j4=.., j5=.., j6=..)
     Копируешь её в pult_pickup_teleop.py, заменяя текущую строку APPROACH.

ПОСЛЕ КАЛИБРОВКИ, ПЕРЕД ЗАПУСКОМ pult_pickup_teleop.py --live:

  7. На руке останавливаешь drag-teach: снова один клик по той же кнопке
     между J5/J6 -- индикатор должен погаснуть.
  8. Верни руку в её "нулевую точку" (zero point, см. PiPER Quick Start
     Manual, раздел 2.2 -- там есть схема). Переключение в CAN-режим
     (что и делает .enable() в PiperArmController) разрешено ТОЛЬКО когда
     рука физически в нулевой точке и drag-teach остановлен -- иначе
     возможно неожиданное резкое движение при включении.
  9. Только теперь можно запускать pult_pickup_teleop.py --live.

Если во время калибровки на экране всё время 0.00 по всем суставам и не
меняется -- см. раздел "Если не работает" в конце файла.

Требуется (не устанавливалось, поставь сам): pip3 install python-can piper_sdk
"""

from __future__ import annotations

import sys
import time
import argparse


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--piper-can", default="can_piper", help="имя поднятого CAN-интерфейса руки")
    ap.add_argument("--rate-hz", type=float, default=10.0, help="частота обновления показаний")
    args = ap.parse_args()

    try:
        from piper_sdk import C_PiperInterface_V2
    except ImportError:
        print("Не найден пакет piper_sdk. Установи: pip3 install python-can piper_sdk")
        sys.exit(1)

    piper = C_PiperInterface_V2(
        can_name=args.piper_can,
        judge_flag=True,
        can_auto_init=True,
        dh_is_offset=1,
    )
    piper.ConnectPort()  # только подписка на чтение CAN, EnableArm() НЕ вызываем

    print(f"Подключено к {args.piper_can} (только чтение, команд движения не шлём).")
    print("Переведи руку в drag-teach (кнопка между J5/J6, один клик, загорится зелёный)")
    print("и вручную подведи захват в позу approach.")
    print("Ctrl+C -- зафиксировать текущую позу.\n")

    period = 1.0 / args.rate_hz
    last_angles = [0.0] * 6

    try:
        while True:
            msg = piper.GetArmJointMsgs()
            j = msg.joint_state
            last_angles = [
                j.joint_1 / 1000.0, j.joint_2 / 1000.0, j.joint_3 / 1000.0,
                j.joint_4 / 1000.0, j.joint_5 / 1000.0, j.joint_6 / 1000.0,
            ]
            line = "  ".join(f"J{i+1}={a:7.2f}°" for i, a in enumerate(last_angles))
            print(f"\r{line}", end="", flush=True)
            time.sleep(period)

    except KeyboardInterrupt:
        j1, j2, j3, j4, j5, j6 = last_angles
        print("\n\n=== Зафиксировано ===")
        print(f"APPROACH = JointAngles(j1={j1:.1f}, j2={j2:.1f}, j3={j3:.1f}, "
              f"j4={j4:.1f}, j5={j5:.1f}, j6={j6:.1f})")
        print("\nВставь эту строку в pult_pickup_teleop.py вместо текущей заглушки APPROACH.")
        print("\nНЕ ЗАБУДЬ перед запуском --live: остановить drag-teach (клик по кнопке")
        print("между J5/J6 ещё раз, индикатор погаснет) и вернуть руку в нулевую точку.")


if __name__ == "__main__":
    main()

# --------------------------------------------------------------------------- #
# Если не работает
# --------------------------------------------------------------------------- #
#
# Углы всё время 0.00 и не меняются:
#   - Проверь, что CAN-интерфейс поднят и называется так же, как в --piper-can
#     (посмотри `ifconfig`, должен быть виден can_piper/can0/... в состоянии UP).
#   - Проверь физическое подключение CAN_H/CAN_L и питание руки (24V).
#   - Если рука настроена в связке master-slave (двурукая конфигурация) -- может
#     потребоваться разово перевести её в slave/read-режим:
#         piper.MasterSlaveConfig(0xFC, 0, 0, 0)
#     Для одиночной руки (как в этом проекте) обычно не требуется.
#
# Ошибка при импорте piper_sdk:
#   - pip3 install python-can piper_sdk (сам пакет и его зависимость).
