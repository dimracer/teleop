#!/usr/bin/env python3
# coding=utf-8
"""
pult_pickup_teleop.py
(rev: пара ModeCtrl+команда; захват без автоподъёма; состояние TURNING)

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
    VRA (крутилка)         -> в режиме JOGGING двигает захват по Z (высота) --
                              это и есть "спуск к предмету": подводи вплотную
                              джойстиком/VRA, отдельного автоспуска перед
                              хватом больше нет (см. grasp_and_lift() ниже)
    SWA -> down (из JOGGING): закрыть захват ПРЯМО ИЗ ТЕКУЩЕЙ (уже подогнанной
                              джойстиком/VRA) позы и ДЕРЖАТЬ -> HOLDING
                              (автоподъёма больше нет: на стенде он выглядел
                              как резкий рывок вверх, убран по согласованию)
    SWA -> down (из HOLDING): РАЗВОРОТ-СБРОС (состояние TURNING, автоматически):
                              J1 -> 150° (180° физически недоступно, лимит
                              PiPER ±154°), спуск через J2/J3 на 40%,
                              отпустить захват, zero point -> IDLE
    SWD -> down (из JOGGING): реально вернуть руку в базовую approach-позу
                              (move_joints(APPROACH)) и заново включить
                              донастройку с нуля (было -- просто "забывало"
                              текущий сдвиг, руку никуда не двигая)

State machine:
    IDLE --SWD--> JOGGING --SWA--> HOLDING --SWA--> TURNING --(авто)--> IDLE
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
APPROACH = JointAngles(j1=0, j2=86, j3=-120, j4=5, j5=28, j6=15)

# насколько мм двигаем захват за один тик поллинга при полном отклонении стика
JOG_MM_PER_TICK = 2.0
JOG_DEADZONE = 8            # из [-100,100], ниже этого -- считаем "стик в нуле"
VRA_DEADZONE = 8


class State(Enum):
    IDLE = auto()
    JOGGING = auto()
    HOLDING = auto()
    TURNING = auto()   # разворот-сброс: J1->150°, спуск J2/J3, отпустить, zero point


class PickupTeleop:
    def __init__(self, arm: PiperArmController, rc: BunkerRcReader, live: bool):
        self.arm = arm
        self.rc = rc
        self.live = live
        self.state = State.IDLE

        rc.on_swd_down(self._on_swd_down)
        rc.on_swa_down(self._on_swa_down)

    # -- переходы состояний --------------------------------------------------

    def _enter_jogging(self, reason: str) -> None:
        """
        Выходит в approach-позу и включает ручную донастройку. Общий код для
        IDLE->JOGGING и для сброса донастройки (повторный SWD внутри JOGGING) --
        раньше сброс НИКУДА физически не двигал руку (просто заново запоминал
        текущую, уже сдвинутую позицию как "ноль"), из-за чего повторный SWD
        выглядел так, будто рука вообще не реагирует. Теперь оба случая явно
        командуют move_joints(APPROACH, wait_settle=True) -- ждут подтверждения
        по фидбеку, что рука ДЕЙСТВИТЕЛЬНО пришла в позу, и только потом
        фиксируют её как базу для джога. wait_settle заодно чинит и другую
        проблему -- раньше поза "на старте джога" бралась ровно через
        фиксированную time.sleep(1.0), и если движение не успевало закончиться
        (например, после долгого перегона от home) -- джог стартовал из
        случайной промежуточной точки, поэтому конечная поза "гуляла" от
        запуска к запуску.

        ОБНОВЛЕНИЕ (исправление "SWD не всегда доводит руку в APPROACH, помогает
        только 2-3-е нажатие"): wait_settle сам по себе только ЖДАЛ прихода в
        позу, но JointCtrl отправлялся один раз -- потерянный CAN-кадр означал
        "рука не поехала вообще", и чинился лишь повторным SWD (повторной
        отправкой). Теперь move_joints(wait_settle=True) сам переотправляет
        команду (до 3 попыток), сверяя реальный режим руки по фидбеку и быстро
        распознавая "рука не стронулась" -- одного нажатия SWD достаточно.
        Если даже после всех ретраев поза не подтверждена (редкий случай:
        обрыв CAN, механическое препятствие) -- по согласованию оставляем
        текущее поведение: входим в JOGGING всё равно, но с error в логе.
        """
        logger.info("%s: выхожу в approach, включаю ручную донастройку", reason)
        settled = self.arm.move_joints(APPROACH, wait_settle=True)
        if not settled:
            logger.error("%s: рука НЕ подтвердила приход в APPROACH даже после ретраев -- "
                          "вхожу в донастройку от ФАКТИЧЕСКОЙ позы; при необходимости нажми SWD ещё раз",
                          reason)
        self.arm.begin_jog()
        self.state = State.JOGGING

    def _on_swd_down(self, s: RcState) -> None:
        if self.state == State.IDLE:
            if self.live and not self.rc.is_stationary():
                logger.warning("SWD игнорирован: база ещё движется (v=%.3f м/с, w=%.3f рад/с)",
                                self.rc.linear_mps or 0.0, self.rc.angular_radps or 0.0)
                return
            self._enter_jogging(">>> IDLE -> JOGGING")

        elif self.state == State.JOGGING:
            self._enter_jogging(">>> сброс донастройки: возврат в approach-позу")

        else:
            logger.info("SWD в состоянии %s игнорируется", self.state)

    def _on_swa_down(self, s: RcState) -> None:
        if self.state == State.JOGGING:
            logger.info(">>> JOGGING -> HOLDING: хват из текущей (уже подогнанной джойстиком) позы, "
                        "без автоподъёма")
            self.arm.grasp_hold()
            self.state = State.HOLDING

        elif self.state == State.HOLDING:
            logger.info(">>> HOLDING -> TURNING: разворот J1, спуск J2/J3, отпустить, zero point")
            self.state = State.TURNING
            try:
                ok = self.arm.turn_lower_release_home(fallback_joints=APPROACH)
                if not ok:
                    logger.error("TURNING: не все шаги подтверждены по фидбеку -- "
                                  "проверь фактическую позу руки")
            finally:
                # в IDLE в любом случае: рука либо в zero point, либо об ошибке
                # уже громко сказано выше; следующий цикл начинается с SWD
                self.state = State.IDLE
            logger.info(">>> TURNING -> IDLE")

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

    logger.info("=== DEMO: подъехали -> approach -> донастройка джойстиком -> захват -> разворот-сброс ===")

    push(count=1)  # состояние покоя

    time.sleep(0.2)
    logger.info("-> SWD down: выход в approach + начало донастройки")
    push(swd=3, count=2)

    logger.info("-> имитирую 5 тиков правого стика вправо-вниз (донастройка X/Y) и крутилки VRA (Z)")
    for i in range(5):
        st = push(swd=3, right_lr=40, right_ud=-30, vra=25, count=3 + i)
        teleop.tick_jog(st)

    time.sleep(0.2)
    logger.info("-> SWA down: захват из текущей (подогнанной) позы, без автоподъёма")
    push(swa=3, swd=3, count=10)

    time.sleep(0.2)
    logger.info("-> SWA down (повторно): разворот J1 -> 150°, спуск J2/J3 на 40%, "
                "отпустить, zero point")
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
