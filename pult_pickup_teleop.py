#!/usr/bin/env python3
# coding=utf-8
"""
pult_pickup_teleop.py

Сценарий "подъехал на штатном пульте Bunker Mini 2.0 -> рукой PiPER взял
предмет", с ручной донастройкой позы захвата под правым джойстиком и
ручкой VRA -- на случай, если база подъехала не идеально в то же место/
под тем же углом, что при калибровке.

Разводка пульта:
    S1, S2, SWB, SWC       -> НЕ ТРОГАЕМ, штатно рулят базой
    SWD -> down (из IDLE)  : выйти в approach-позу и войти в режим ручной
                              донастройки (JOGGING)
    правый стик (лево/право, верх/низ) -> в режиме JOGGING двигает захват
                              по X/Y (мм за тик, с зоной нечувствительности)
    VRA (крутилка)         -> в режиме JOGGING двигает захват по Z (высота)
    SWA -> down (из JOGGING): выполнить спуск-захват-подъём из ТЕКУЩЕЙ
                              (уже подогнанной) позы -> состояние HOLDING
    SWA -> down (из HOLDING): отпустить предмет, вернуть руку в home ->
                              состояние IDLE
    SWD -> down (из JOGGING): сбросить донастройку, вернуться к базовой
                              approach-позе (переснять офсет в ноль)

State machine:
    IDLE --SWD--> JOGGING --SWA--> HOLDING --SWA--> IDLE
              ^______________SWD (reset)_____|

БЕЗОПАСНОСТЬ:
    - Перед стартом approach/захвата на реальном железе (не dry-run)
      проверяется rc.is_stationary() -- база должна физически стоять
      (пульт всегда имеет приоритет над CAN, программно её не остановить,
      см. bunker_rc_reader.py).
    - Донастройка ограничена безопасной "коробкой" вокруг approach-позы
      (PiperArmController.JOG_XY_LIMIT_MM/JOG_Z_LIMIT_MM) -- увести руку
      далеко в сторону джойстиком не получится.

ЗАГЛУШКИ, ТРЕБУЮЩИЕ КАЛИБРОВКИ: APPROACH ниже -- joint-поза "рука висит
над предметом издалека", откалибровать под свой стол через drag-teach
(см. docstring PiperArmController.pickup_sequence в agilex_platform.py).
Дальше донастройка идёт уже в декартовых координатах поверх этой позы.

Запуск на реальном стенде:
    python3 pult_pickup_teleop.py --live --bunker-can can_bunker --piper-can can_piper

Проверка логики без железа (эмулирует нажатия пульта и джойстик):
    python3 pult_pickup_teleop.py --demo
"""

from __future__ import annotations

import time
import logging
import argparse
from enum import Enum, auto

from agilex_platform import PiperArmController, JointAngles, EndPose
from bunker_rc_reader import BunkerRcReader, RcState

logger = logging.getLogger("pult_pickup_teleop")
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

# ЗАГЛУШКА -- обязательно откалибровать под свой стол/предмет
APPROACH = JointAngles(j1=0, j2=40, j3=-60, j4=0, j5=20, j6=0)

# насколько мм двигаем захват за один тик поллинга при полном отклонении стика
JOG_MM_PER_TICK = 2.0
JOG_DEADZONE = 8            # из [-100,100], ниже этого -- считаем "стик в нуле"
VRA_DEADZONE = 8


class State(Enum):
    IDLE = auto()
    JOGGING = auto()
    HOLDING = auto()


class PickupTeleop:
    def __init__(self, arm: PiperArmController, rc: BunkerRcReader, live: bool):
        self.arm = arm
        self.rc = rc
        self.live = live
        self.state = State.IDLE

        rc.on_swd_down(self._on_swd_down)
        rc.on_swa_down(self._on_swa_down)

    # -- переходы состояний --------------------------------------------------

    def _on_swd_down(self, s: RcState) -> None:
        if self.state == State.IDLE:
            if self.live and not self.rc.is_stationary():
                logger.warning("SWD игнорирован: база ещё движется (v=%.3f м/с, w=%.3f рад/с)",
                                self.rc.linear_mps or 0.0, self.rc.angular_radps or 0.0)
                return
            logger.info(">>> IDLE -> JOGGING: выхожу в approach, включаю ручную донастройку")
            self.arm.move_joints(APPROACH)
            time.sleep(1.0)
            self.arm.begin_jog()
            self.state = State.JOGGING

        elif self.state == State.JOGGING:
            logger.info(">>> сброс донастройки к базовой approach-позе")
            self.arm.begin_jog()

        else:
            logger.info("SWD в состоянии %s игнорируется", self.state)

    def _on_swa_down(self, s: RcState) -> None:
        if self.state == State.JOGGING:
            logger.info(">>> JOGGING -> HOLDING: спуск-захват-подъём из текущей позы")
            self.arm.descend_grasp_lift()
            self.state = State.HOLDING

        elif self.state == State.HOLDING:
            logger.info(">>> HOLDING -> IDLE: отпускаю предмет, домой")
            self.arm.release_and_home()
            self.state = State.IDLE

        else:
            logger.info("SWA в состоянии %s игнорируется (нечего отпускать)", self.state)

    # -- непрерывный джог по стику/крутилке ----------------------------------

    def tick_jog(self, rc_state: RcState) -> None:
        if self.state != State.JOGGING:
            return

        dx = _apply_deadzone(rc_state.right_lr, JOG_DEADZONE) / 100.0 * JOG_MM_PER_TICK
        dy = _apply_deadzone(rc_state.right_ud, JOG_DEADZONE) / 100.0 * JOG_MM_PER_TICK
        dz = _apply_deadzone(rc_state.vra, VRA_DEADZONE) / 100.0 * JOG_MM_PER_TICK

        if dx == 0.0 and dy == 0.0 and dz == 0.0:
            return

        target = self.arm.apply_jog(dx_mm=dx, dy_mm=dy, dz_mm=dz)
        logger.debug("jog -> x=%.1f y=%.1f z=%.1f", target.x, target.y, target.z)


def _apply_deadzone(v: int, deadzone: int) -> float:
    if abs(v) < deadzone:
        return 0.0
    return float(v)


def run_live(bunker_can: str, piper_can: str, poll_timeout: float) -> None:
    arm = PiperArmController(can_name=piper_can, dry_run=False)
    rc = BunkerRcReader(can_name=bunker_can, dry_run=False)
    teleop = PickupTeleop(arm, rc, live=True)

    arm.connect()
    arm.enable()
    arm.move_home()
    rc.start()

    logger.info("Готов. SWD=down -> approach+донастройка (правый стик X/Y, VRA=Z), "
                "SWA=down -> взять/отпустить. Ctrl+C для выхода.")
    try:
        while True:
            state = rc.poll_once(timeout=poll_timeout)
            if state is not None:
                teleop.tick_jog(state)
    except KeyboardInterrupt:
        logger.info("Останов по Ctrl+C")
    finally:
        arm.move_home()
        arm.disable()
        rc.stop()


def run_demo() -> None:
    """Прогоняет всю цепочку (approach -> jog -> grasp -> release) на синтетических данных, без CAN."""
    arm = PiperArmController(can_name="can_piper", dry_run=True)
    rc = BunkerRcReader(can_name="can_bunker", dry_run=True)
    teleop = PickupTeleop(arm, rc, live=False)

    arm.connect()
    arm.enable()
    arm.move_home()

    def frame(swa=2, swb=2, swc=2, swd=2, right_lr=0, right_ud=0, left_ud=0, left_lr=0, vra=0, count=0) -> bytes:
        b0 = (swa & 0b11) | ((swb & 0b11) << 2) | ((swc & 0b11) << 4) | ((swd & 0b11) << 6)

        def s8(v: int) -> int:
            return v & 0xFF

        return bytes([b0, s8(right_lr), s8(right_ud), s8(left_ud), s8(left_lr), s8(vra), 0, count])

    def push(**kwargs) -> RcState:
        st = RcState.from_can_data(frame(**kwargs), time.time())
        rc._process_state(st)
        return st

    logger.info("=== DEMO: подъехали -> approach -> ручная донастройка джойстиком -> захват -> отпустить ===")

    push(count=1)  # состояние покоя

    time.sleep(0.2)
    logger.info("-> SWD down: выход в approach + начало донастройки")
    push(swd=3, count=2)

    logger.info("-> имитирую 5 тиков правого стика вправо-вниз (донастройка X/Y) и крутилки VRA (Z)")
    for i in range(5):
        st = push(swd=3, right_lr=40, right_ud=-30, vra=25, count=3 + i)
        teleop.tick_jog(st)

    time.sleep(0.2)
    logger.info("-> SWA down: захват из текущей (подогнанной) позы")
    push(swa=3, swd=3, count=10)

    time.sleep(0.2)
    logger.info("-> SWA down: отпустить и вернуться домой")
    push(swa=2, swd=3, count=11)  # сначала отпускаем SWA (up), чтобы поймать следующий edge
    push(swa=3, swd=3, count=12)

    logger.info("=== DEMO завершено ===")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bunker-can", default="can_bunker")
    ap.add_argument("--piper-can", default="can_piper")
    ap.add_argument("--poll-timeout", type=float, default=0.05)
    ap.add_argument("--live", action="store_true", help="реальная работа с железом")
    ap.add_argument("--demo", action="store_true", help="прогон логики на синтетических данных, без CAN")
    args = ap.parse_args()

    if args.demo:
        run_demo()
    elif args.live:
        run_live(args.bunker_can, args.piper_can, args.poll_timeout)
    else:
        logger.info("Ничего не выбрано: используй --demo (проверка логики) или --live (реальное железо).")


if __name__ == "__main__":
    main()
