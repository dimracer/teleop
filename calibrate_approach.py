#!/usr/bin/env python3
# coding=utf-8
"""
calibrate_approach.py

Помогает подобрать APPROACH для pult_pickup_teleop.py: подключается к PiPER
ТОЛЬКО НА ЧТЕНИЕ (не включает CAN-режим, не шлёт команд движения) и в
реальном времени печатает текущие углы суставов, пока ты вручную двигаешь
руку в режиме drag-teach. Когда рука встанет в нужную позу над предметом —
жмёшь Ctrl+C, скрипт печатает готовую строку для вставки в pult_pickup_teleop.py.

ДОПОЛНИТЕЛЬНО: режим --set-zero -- переустановка нулевой точки руки.
Нужен, если ноль руки "уехал" (симптом: при запуске pult_pickup_teleop.py
рука едет в неправильный "ноль", например J3 уходит сильно вниз, лог при
этом чистый -- фидбек честно считает эту позу нулём). Ноль может сместиться
после удара/перегрузки (проскочил ремень/редуктор) или из-за случайной
переустановки нуля кнопкой на руке. Порядок:

  1. Останови pult_pickup_teleop.py, руку оставь под питанием.
  2. Включи drag-teach (один клик по кнопке между J5/J6, ровный зелёный).
  3. Руками выставь руку ТОЧНО в нулевую позу (zero point, схема в PiPER
     Quick Start Manual, раздел 2.2) и держи её.
  4. Запусти:  python3 calibrate_approach.py --piper-can can_piper --set-zero
     Скрипт покажет, что рука СЕЙЧАС считает своими углами (там и будет
     виден увод, например J3 = -30° вместо 0), затем спросит подтверждение.
  5. Набери ZERO и Enter -- текущая поза будет записана как ноль всех суставов
     (JointConfig(7, 0xAE)).
  6. Останови drag-teach (клик по кнопке, индикатор погас), сними/подай
     питание руки (power-cycle) и проверь: python3 calibrate_approach.py --
     в нулевой позе все суставы должны показывать ~0.00.
  7. ВАЖНО: после переустановки нуля прежняя калибровка APPROACH могла
     опираться на старый (сдвинутый) ноль -- проверь позу APPROACH и при
     необходимости перекалибруй её заново (обычный режим этого скрипта).

ЕСЛИ --set-zero НЕ ПОМОГАЕТ (рука "не запоминает" ноль, суставы едут в
неправильную сторону, фидбек ~0 при ненулевой физической позе) -- это
известный баг инициализации joint states прошивкой после power-cycle:
https://github.com/agilexrobotics/piper_sdk/issues/35
Симптомы там ровно эти: рука едет в "неправильный случайный ноль",
установка нулей через SDK не помогает, привязка углов ломается (движение
одного сустава меняет показания других). Лечение -- режим --fix-mapping
этого скрипта: Master-режим -> обратно Slave -> reset -> power-cycle.
Если повторяется после каждого power-cycle -- нужна прошивка: напиши в
support@agilex.ai, приложив версию прошивки (видно в ArmRobotUA).

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


def _read_angles(piper) -> list[float]:
    msg = piper.GetArmJointMsgs()
    j = msg.joint_state
    return [
        j.joint_1 / 1000.0, j.joint_2 / 1000.0, j.joint_3 / 1000.0,
        j.joint_4 / 1000.0, j.joint_5 / 1000.0, j.joint_6 / 1000.0,
    ]


def _fmt(angles: list[float]) -> str:
    return "  ".join(f"J{i+1}={a:7.2f}°" for i, a in enumerate(angles))


def run_set_zero(piper) -> None:
    """
    Переустановка нулевой точки: текущая ФИЗИЧЕСКАЯ поза руки будет записана
    как ноль всех суставов (JointConfig(7, 0xAE)). Использовать, когда ноль
    руки "уехал" (после удара/проскока ремня) -- см. docstring файла, шаги 1-7.
    """
    print("\n=== ПЕРЕУСТАНОВКА НУЛЕВОЙ ТОЧКИ ===")
    print("Рука должна быть в drag-teach и ФИЗИЧЕСКИ выставлена в нулевую позу")
    print("(zero point, схема -- PiPER Quick Start Manual, раздел 2.2).\n")
    print("Сейчас рука считает своими углами (если ноль уехал -- тут будет виден увод):")
    for _ in range(3):
        print(f"\r  {_fmt(_read_angles(piper))}", end="", flush=True)
        time.sleep(0.5)
    print("\n\nВНИМАНИЕ: текущая поза станет НУЛЁМ для всех 6 суставов.")
    print("Отменить это можно только повторной переустановкой нуля.")
    answer = input("Набери ZERO и Enter для подтверждения (что-либо другое -- отмена): ").strip()
    if answer != "ZERO":
        print("Отменено, ничего не отправлено.")
        return

    # 7 = все суставы, 0xAE = установить текущую позицию как ноль
    # (позиционные аргументы -- имена kwargs отличаются между версиями piper_sdk)
    piper.JointConfig(7, 0xAE)
    time.sleep(0.5)
    print("\nКоманда отправлена. Проверка -- рука теперь считает:")
    print(f"  {_fmt(_read_angles(piper))}")
    print("\nДальше: останови drag-teach (клик по кнопке J5/J6), сделай power-cycle руки,")
    print("затем проверь нули этим же скриптом без --set-zero (в нулевой позе ~0.00).")
    print("И НЕ ЗАБУДЬ перепроверить/перекалибровать APPROACH -- он мог опираться на старый ноль.")


def run_fix_mapping(piper) -> None:
    """
    Восстановление привязки углов суставов -- workaround известного бага
    прошивки (https://github.com/agilexrobotics/piper_sdk/issues/35):
    после power-cycle рука может неправильно инициализировать joint states.
    Симптомы: фидбек ~0 при ненулевой физической позе, движение "к нулю"
    идёт в неправильную сторону, --set-zero не запоминается, движение
    одного сустава меняет показания других. Workaround из issue #35:
    переключить руку в Master-режим, обратно в Slave, сделать reset,
    затем power-cycle.
    """
    print("\n=== ВОССТАНОВЛЕНИЕ ПРИВЯЗКИ УГЛОВ (workaround piper_sdk issue #35) ===")
    print("ВНИМАНИЕ: при переключении в Master-режим моторы могут стать податливыми --")
    print("ПРИДЕРЖИВАЙ руку рукой, чтобы она не упала под собственным весом.")
    answer = input("Готов? Набери FIX и Enter (что-либо другое -- отмена): ").strip()
    if answer != "FIX":
        print("Отменено, ничего не отправлено.")
        return

    print(f"\nУглы ДО:    {_fmt(_read_angles(piper))}")

    print("1/3: переключаю в Master (teaching input)...")
    piper.MasterSlaveConfig(0xFA, 0, 0, 0)
    time.sleep(2.0)

    print("2/3: возвращаю в Slave (motion output)...")
    piper.MasterSlaveConfig(0xFC, 0, 0, 0)
    time.sleep(2.0)

    print("3/3: reset...")
    try:
        piper.MotionCtrl_1(0x02, 0, 0)   # 0x02 = resume/reset
    except Exception as e:
        print(f"  MotionCtrl_1 reset не отправился ({e}) -- не критично, "
              "главное было master/slave-переключение")
    time.sleep(1.0)

    print(f"Углы ПОСЛЕ: {_fmt(_read_angles(piper))}")
    print("\nДальше обязательно:")
    print("  1. Power-cycle руки (сними и подай 24В).")
    print("  2. Проверь без флагов (python3 calibrate_approach.py): в физическом нуле")
    print("     все суставы ~0.00, и при движении сустава РУКАМИ (drag-teach) меняется")
    print("     именно соответствующий J на экране.")
    print("  3. Если привязка снова ломается после power-cycle -- это прошивка:")
    print("     напиши в support@agilex.ai, приложи версию прошивки (ArmRobotUA).")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--piper-can", default="can_piper", help="имя поднятого CAN-интерфейса руки")
    ap.add_argument("--rate-hz", type=float, default=10.0, help="частота обновления показаний")
    ap.add_argument("--set-zero", action="store_true",
                    help="переустановить нулевую точку: текущая поза руки станет нулём "
                         "всех суставов (см. порядок действий в docstring)")
    ap.add_argument("--fix-mapping", action="store_true",
                    help="восстановить привязку углов суставов (Master->Slave->reset, "
                         "workaround piper_sdk issue #35; см. docstring)")
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
    piper.ConnectPort()  # подписка на чтение CAN; EnableArm()/команды движения НЕ шлём

    if args.fix_mapping:
        run_fix_mapping(piper)
        return

    if args.set_zero:
        run_set_zero(piper)
        return

    print(f"Подключено к {args.piper_can} (только чтение, команд движения не шлём).")
    print("Переведи руку в drag-teach (кнопка между J5/J6, один клик, загорится зелёный)")
    print("и вручную подведи захват в позу approach.")
    print("Ctrl+C -- зафиксировать текущую позу.\n")

    period = 1.0 / args.rate_hz
    last_angles = [0.0] * 6

    try:
        while True:
            last_angles = _read_angles(piper)
            print(f"\r{_fmt(last_angles)}", end="", flush=True)
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
