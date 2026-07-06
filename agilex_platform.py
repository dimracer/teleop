#!/usr/bin/env python3
# coding=utf-8
"""
agilex_platform.py

Прямая (без ROS/ROS2) интеграция двух роботов AgileX на одном CAN-хосте:

  - Bunker Mini 2.0   -> официальный пакет `pyagxrobots`
                         (https://github.com/agilexrobotics/pyagxrobots)
  - PiPER (6-DOF рука) -> официальный `piper_sdk`
                         (https://github.com/agilexrobotics/piper_sdk)

Обе библиотеки общаются с железом напрямую через python-can (socketcan),
без ROS/ROS2 узлов и без rosbridge.

ТРЕБОВАНИЯ (НЕ устанавливались автоматически -- установи вручную, когда будешь готов):
    pip3 install python-can piper_sdk pyagxrobots

ЖЕЛЕЗО / CAN:
    У Bunker Mini и PiPER разные протоколы поверх CAN, но одинаковый физический
    интерфейс (USB-CAN adapter, socketcan). Штатная схема AgileX при совместном
    использовании базы и руки -- ДВА отдельных USB-CAN адаптера, каждый поднят как
    свой интерфейс (например can_bunker и can_piper), а не общая шина.

    Bunker Mini 2.0:  бод 500000
    PiPER:             бод 1000000   (менять нельзя, это фиксировано в прошивке)

    Активация (пример, имена интерфейсов подставь свои):
        bash find_all_can_port.sh                       # узнать USB-порт каждого адаптера
        bash can_activate.sh can_bunker 500000  "<usb-port-bunker>"
        bash can_activate.sh can_piper  1000000 "<usb-port-piper>"

    Или для активации сразу обоих: can_muti_activate.sh из репозитория piper_sdk.

БЕЗОПАСНОСТЬ:
    - PiPER включается через EnableArm() только по явному вызову .enable().
    - Bunker перед выдачей скорости требует EnableCAN() -- тоже явный вызов.
    - dry_run=True (по умолчанию в демо ниже) не шлёт реальных команд в CAN,
      только логирует -- полезно, пока стенд не подключён физически.

ВАЖНОЕ ИЗМЕНЕНИЕ (после разбора инцидента с "рука не двигается через раз"):
    enable()/enable_joint_mode()/enable_end_pose_mode() теперь не просто
    один раз шлют команду и надеются на лучшее -- они ПОДТВЕРЖДАЮТ результат
    по обратной связи (GetArmEnableStatus()/GetArmStatus()) и повторяют
    попытку при неудаче, громко сообщая об этом в лог. Раньше при потере
    одного-единственного CAN-кадра EnableArm()/ModeCtrl() (например, сразу
    после power-cycle руки) код молча считал, что всё включилось, и дальше
    исправно "выполнял" сценарий в логах, хотя физически рука ничего не
    получала. Подробности -- в README, раздел "Диагностика: рука не
    реагирует не при каждом запуске".
"""

from __future__ import annotations

import time
import logging
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger("agilex_platform")
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")


# --------------------------------------------------------------------------- #
# Bunker Mini 2.0 -- мобильная база
# --------------------------------------------------------------------------- #

class BunkerMiniController:
    """
    Обёртка над pyagxrobots.pysdkugv.BunkerBase.
    """

    MAX_LINEAR_MPS = 0.5
    MAX_ANGULAR_RPS = 0.78
    MAX_PAYLOAD_KG = 25.0  # актуально для Bunker Mini 2.0

    def __init__(self, can_name: str = "can_bunker", dry_run: bool = True):
        self.can_name = can_name
        self.dry_run = dry_run
        self._base = None
        self._enabled = False

        if not dry_run:
            import pyagxrobots  # локальный импорт: не требуем пакет в dry_run
            self._base = pyagxrobots.pysdkugv.BunkerBase()
        else:
            logger.info("[Bunker] dry_run=True: реального подключения к %s не будет", can_name)

    def enable(self) -> None:
        if self.dry_run:
            logger.info("[Bunker] (dry_run) EnableCAN()")
        else:
            self._base.EnableCAN()
        self._enabled = True
        logger.info("[Bunker] enabled")

    def set_velocity(self, linear_mps: float = 0.0, angular_rps: float = 0.0) -> None:
        if not self._enabled:
            raise RuntimeError("Bunker не включён: вызови enable() перед движением")

        linear_mps = max(-self.MAX_LINEAR_MPS, min(self.MAX_LINEAR_MPS, linear_mps))
        angular_rps = max(-self.MAX_ANGULAR_RPS, min(self.MAX_ANGULAR_RPS, angular_rps))

        if self.dry_run:
            logger.info("[Bunker] (dry_run) SetMotionCommand(linear=%.3f, angular=%.3f)",
                        linear_mps, angular_rps)
            return
        self._base.SetMotionCommand(linear_vel=linear_mps, angular_vel=angular_rps)

    def stop(self) -> None:
        self.set_velocity(0.0, 0.0)

    def get_state(self) -> dict:
        if self.dry_run:
            return {"linear_vel": 0.0, "angular_vel": 0.0, "battery_v": None}
        return {
            "linear_vel": self._base.GetLinearVelocity(),
            "angular_vel": self._base.GetAngularVelocity(),
            "battery_v": self._base.GetBatteryVoltage(),
        }


# --------------------------------------------------------------------------- #
# PiPER -- 6-DOF рука
# --------------------------------------------------------------------------- #

@dataclass
class JointAngles:
    j1: float = 0.0
    j2: float = 0.0
    j3: float = 0.0
    j4: float = 0.0
    j5: float = 0.0
    j6: float = 0.0

    def as_millidegrees(self) -> tuple[int, int, int, int, int, int]:
        return tuple(int(round(v * 1000)) for v in
                     (self.j1, self.j2, self.j3, self.j4, self.j5, self.j6))


@dataclass
class EndPose:
    """Декартова поза TCP: x/y/z в мм, rx/ry/rz в градусах (Эйлер), см. GetArmEndPoseMsgs/EndPoseCtrl."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0

    def as_milli(self) -> tuple[int, int, int, int, int, int]:
        return tuple(int(round(v * 1000)) for v in
                     (self.x, self.y, self.z, self.rx, self.ry, self.rz))


class PiperArmController:
    """
    Обёртка над piper_sdk.C_PiperInterface_V2 (интерфейс V2, прошивка >= V1.5-2).

    Поддерживает два режима движения:
      - joint-режим (MOVE J)  -- move_joints()/move_home()/pickup_sequence()
      - end-pose режим (MOVE L) -- move_to_pose()/begin_jog()/apply_jog(),
        нужен для ручной донастройки положения захвата перед взятием
        (когда база подъехала под другим углом/местом, чем при калибровке).

    Класс сам переключает ModeCtrl между joint/end-pose при вызове
    соответствующих методов -- вручную дёргать ModeCtrl не нужно.

    enable()/enable_joint_mode()/enable_end_pose_mode() ПОДТВЕРЖДАЮТ результат
    по фидбеку (GetArmEnableStatus()/GetArmStatus()) с повтором попыток --
    см. docstring enable() ниже, почему это важно.
    """

    JOINT_LIMITS_DEG = {
        1: (-150.0, 150.0),
        2: (0.0, 180.0),
        3: (-170.0, 0.0),
        4: (-100.0, 100.0),
        5: (-70.0, 70.0),
        6: (-120.0, 120.0),
    }

    # безопасная "коробка" ручного джога вокруг базовой позы (approach), мм
    JOG_XY_LIMIT_MM = 40.0
    JOG_Z_LIMIT_MM = 50.0

    # сколько раз повторять EnableArm()/ModeCtrl(), если фидбек не подтвердил
    ENABLE_RETRIES = 3
    MODE_RETRIES = 3
    CONFIRM_TIMEOUT_S = 1.5   # сколько ждём подтверждения после каждой попытки
    CONFIRM_POLL_S = 0.05

    # ожидание физического прихода в целевую joint-позу (move_joints(..., wait_settle=True))
    MOVE_SETTLE_TIMEOUT_S = 5.0
    MOVE_SETTLE_POLL_S = 0.1
    MOVE_SETTLE_TOL_DEG = 1.5

    # как часто во время джога перепроверяем реальный режим руки по фидбеку
    # (а не просто доверяем закэшированному self._move_mode) -- см. apply_jog()
    JOG_MODE_RECHECK_S = 0.5

    def __init__(self, can_name: str = "can_piper", dry_run: bool = True):
        self.can_name = can_name
        self.dry_run = dry_run
        self._piper = None
        self._enabled = False
        self._move_mode: Optional[str] = None   # "joint" | "end_pose" | None
        self._jog_base: Optional[EndPose] = None
        self._jog_offset = {"x": 0.0, "y": 0.0, "z": 0.0}
        self._last_mode_check = 0.0

        if not dry_run:
            from piper_sdk import C_PiperInterface_V2
            self._piper = C_PiperInterface_V2(
                can_name=can_name,
                judge_flag=True,
                can_auto_init=True,
                dh_is_offset=1,
                start_sdk_joint_limit=True,
                start_sdk_gripper_limit=True,
            )
        else:
            logger.info("[PiPER] dry_run=True: реального подключения к %s не будет", can_name)

    def connect(self) -> None:
        if self.dry_run:
            logger.info("[PiPER] (dry_run) ConnectPort()")
            return
        self._piper.ConnectPort()

    # -- внутренние помощники: ждать подтверждения по фидбеку ---------------

    def _wait_for(self, predicate) -> bool:
        """Опрашивает predicate() до CONFIRM_TIMEOUT_S, пока не вернёт True."""
        deadline = time.time() + self.CONFIRM_TIMEOUT_S
        while time.time() < deadline:
            try:
                if predicate():
                    return True
            except Exception:
                logger.exception("[PiPER] ошибка при проверке фидбека (см. трассировку)")
            time.sleep(self.CONFIRM_POLL_S)
        return False

    def _ctrl_mode_feedback_is(self, ctrl_mode: int, move_mode: int) -> bool:
        fb = self._piper.GetArmStatus()
        st = fb.arm_status
        return st.ctrl_mode == ctrl_mode and st.mode_feed == move_mode

    def _all_motors_enabled(self) -> bool:
        status = self._piper.GetArmEnableStatus()
        return bool(status) and all(status)

    # -- включение/выключение -------------------------------------------------

    def enable(self) -> None:
        """
        ВАЖНО (см. официальный PiPER Quick Start Manual, раздел 2.2):
        переключение в CAN-режим управления разрешено ТОЛЬКО когда рука
        физически находится в нулевой точке и drag-teach остановлен
        (индикатор между J5/J6 не горит). Эта проверка не автоматизирована --
        перед первым enable() на реальном стенде убедись в этом вручную.

        В остальном метод сам:
          1. Явно фиксирует роль "исполнительная рука" (motion output arm,
             MasterSlaveConfig(0xFC,...)) -- чтобы исключить неоднозначность
             master/slave-конфигурации, оставшуюся с прошлых сессий.
          2. Шлёт EnableArm() и ждёт подтверждения по GetArmEnableStatus()
             (все 6 моторов enabled). Если не подтвердилось -- повторяет
             до ENABLE_RETRIES раз, иначе поднимает RuntimeError вместо
             того чтобы молча считать, что всё включилось.
        """
        if self.dry_run:
            logger.info("[PiPER] (dry_run) EnableArm(7)")
            self._enabled = True
            self._move_mode = None
            self.enable_joint_mode()
            logger.info("[PiPER] enabled")
            return

        try:
            self._piper.MasterSlaveConfig(0xFC, 0, 0, 0)
        except Exception:
            logger.exception("[PiPER] MasterSlaveConfig(0xFC,...) не удался (не критично, продолжаю)")

        confirmed = False
        for attempt in range(1, self.ENABLE_RETRIES + 1):
            try:
                self._piper.EnableArm(motor_num=7, enable_flag=0x02)
            except Exception:
                logger.exception("[PiPER] EnableArm() попытка %d/%d не удалась", attempt, self.ENABLE_RETRIES)

            if self._wait_for(self._all_motors_enabled):
                confirmed = True
                break
            logger.warning("[PiPER] enable не подтверждён по GetArmEnableStatus() "
                            "(попытка %d/%d) -- повторяю", attempt, self.ENABLE_RETRIES)

        if not confirmed:
            raise RuntimeError(
                "PiPER: не удалось подтвердить включение моторов после "
                f"{self.ENABLE_RETRIES} попыток (GetArmEnableStatus() так и не стал all-True). "
                "Проверь: рука в нулевой точке? drag-teach точно остановлен (индикатор погашен, "
                "не мигает)? физическое подключение CAN_H/CAN_L и питание 24В? "
                "См. README, раздел про диагностику."
            )

        self._enabled = True
        self._move_mode = None  # сбрасываем, чтобы ModeCtrl ниже точно переотправился и был подтверждён
        self.enable_joint_mode()
        logger.info("[PiPER] enabled (подтверждено по GetArmEnableStatus)")

    def disable(self) -> None:
        if self.dry_run:
            logger.info("[PiPER] (dry_run) DisableArm(7)")
        else:
            try:
                self._piper.DisableArm(motor_num=7, enable_flag=0x01)
            except Exception:
                logger.exception("[PiPER] DisableArm() не удался")
        self._enabled = False
        self._move_mode = None
        logger.info("[PiPER] disabled")

    # -- переключение режима движения --------------------------------------

    def enable_joint_mode(self, speed_pct: int = 30) -> None:
        """MOVE J -- нужен перед move_joints()/move_home(). Подтверждается по GetArmStatus()."""
        if self._move_mode == "joint":
            return
        if self.dry_run:
            logger.info("[PiPER] (dry_run) ModeCtrl(MOVE J, speed=%d%%)", speed_pct)
            self._move_mode = "joint"
            return

        confirmed = False
        for attempt in range(1, self.MODE_RETRIES + 1):
            try:
                self._piper.ModeCtrl(ctrl_mode=0x01, move_mode=0x01, move_spd_rate_ctrl=speed_pct)
            except Exception:
                logger.exception("[PiPER] ModeCtrl(MOVE J) попытка %d/%d не удалась", attempt, self.MODE_RETRIES)

            if self._wait_for(lambda: self._ctrl_mode_feedback_is(0x01, 0x01)):
                confirmed = True
                break
            logger.warning("[PiPER] переход в MOVE J не подтверждён по GetArmStatus() "
                            "(попытка %d/%d) -- повторяю", attempt, self.MODE_RETRIES)

        if not confirmed:
            raise RuntimeError(
                "PiPER: не удалось подтвердить переход в MOVE J после "
                f"{self.MODE_RETRIES} попыток (GetArmStatus().ctrl_mode/mode_feed не совпал). "
                "Проверь состояние руки и CAN-подключение."
            )
        self._move_mode = "joint"

    def enable_end_pose_mode(self, speed_pct: int = 20) -> None:
        """MOVE L -- нужен перед move_to_pose()/begin_jog()/apply_jog(). Подтверждается по GetArmStatus()."""
        if self._move_mode == "end_pose":
            return
        if self.dry_run:
            logger.info("[PiPER] (dry_run) ModeCtrl(MOVE L, speed=%d%%)", speed_pct)
            self._move_mode = "end_pose"
            return

        confirmed = False
        for attempt in range(1, self.MODE_RETRIES + 1):
            try:
                self._piper.ModeCtrl(ctrl_mode=0x01, move_mode=0x02, move_spd_rate_ctrl=speed_pct)
            except Exception:
                logger.exception("[PiPER] ModeCtrl(MOVE L) попытка %d/%d не удалась", attempt, self.MODE_RETRIES)

            if self._wait_for(lambda: self._ctrl_mode_feedback_is(0x01, 0x02)):
                confirmed = True
                break
            logger.warning("[PiPER] переход в MOVE L не подтверждён по GetArmStatus() "
                            "(попытка %d/%d) -- повторяю", attempt, self.MODE_RETRIES)

        if not confirmed:
            raise RuntimeError(
                "PiPER: не удалось подтвердить переход в MOVE L после "
                f"{self.MODE_RETRIES} попыток (GetArmStatus().ctrl_mode/mode_feed не совпал). "
                "Проверь состояние руки и CAN-подключение."
            )
        self._move_mode = "end_pose"

    def _clip(self, joints: JointAngles) -> JointAngles:
        clipped = {}
        for i, name in enumerate(("j1", "j2", "j3", "j4", "j5", "j6"), start=1):
            lo, hi = self.JOINT_LIMITS_DEG[i]
            v = getattr(joints, name)
            clipped[name] = max(lo, min(hi, v))
        return JointAngles(**clipped)

    def move_joints(self, joints: JointAngles, wait_settle: bool = False) -> None:
        """
        wait_settle=True: не возвращаться, пока рука реально не придёт в целевую
        позу (по фидбеку GetArmJointMsgs(), с допуском MOVE_SETTLE_TOL_DEG и
        таймаутом MOVE_SETTLE_TIMEOUT_S) -- вместо того чтобы гадать фиксированной
        задержкой time.sleep(...). Это важно перед тем, как что-то ещё читает
        текущую позу руки (например begin_jog() в pult_pickup_teleop.py) --
        иначе можно случайно снять позу "на лету", в середине движения, и
        каждый раз получать чуть разную точку (см. README, диагностика).
        """
        if not self._enabled:
            raise RuntimeError("PiPER не включён: вызови enable() перед движением")
        self.enable_joint_mode()

        joints = self._clip(joints)
        j1, j2, j3, j4, j5, j6 = joints.as_millidegrees()

        if self.dry_run:
            logger.info("[PiPER] (dry_run) JointCtrl(%s)", (j1, j2, j3, j4, j5, j6))
            return

        try:
            self._piper.JointCtrl(j1, j2, j3, j4, j5, j6)
        except Exception:
            logger.exception("[PiPER] JointCtrl(%s) попытка 1/2 не удалась -- повторяю",
                              (j1, j2, j3, j4, j5, j6))
            try:
                self._piper.JointCtrl(j1, j2, j3, j4, j5, j6)
            except Exception:
                logger.exception("[PiPER] JointCtrl(%s) не удался -- команда НЕ ушла в CAN",
                                  (j1, j2, j3, j4, j5, j6))
                raise

        if wait_settle:
            deadline = time.time() + self.MOVE_SETTLE_TIMEOUT_S
            while time.time() < deadline:
                if self._joints_close(joints):
                    return
                time.sleep(self.MOVE_SETTLE_POLL_S)
            logger.warning(
                "[PiPER] move_joints: рука не подтвердила приход в целевую позу за %.1fс "
                "(GetArmJointMsgs() так и не сошёлся в пределах %.1f°) -- продолжаю всё равно, "
                "но следующая captured поза может быть не окончательной",
                self.MOVE_SETTLE_TIMEOUT_S, self.MOVE_SETTLE_TOL_DEG,
            )

    def move_home(self) -> None:
        self.move_joints(JointAngles(0, 0, 0, 0, 0, 0), wait_settle=True)

    # -- декартовы движения (MOVE L) для ручной донастройки -----------------

    def get_end_pose_mm_deg(self) -> Optional[EndPose]:
        """Читает текущую позу TCP по фидбеку. В dry_run -- None (нет реального фидбека)."""
        if self.dry_run:
            return None
        msg = self._piper.GetArmEndPoseMsgs()
        ep = msg.end_pose
        return EndPose(
            x=ep.X_axis / 1000.0, y=ep.Y_axis / 1000.0, z=ep.Z_axis / 1000.0,
            rx=ep.RX_axis / 1000.0, ry=ep.RY_axis / 1000.0, rz=ep.RZ_axis / 1000.0,
        )

    def get_joint_angles_deg(self) -> Optional[JointAngles]:
        """Читает текущие углы суставов по фидбеку. В dry_run -- None."""
        if self.dry_run:
            return None
        msg = self._piper.GetArmJointMsgs()
        j = msg.joint_state
        return JointAngles(
            j1=j.joint_1 / 1000.0, j2=j.joint_2 / 1000.0, j3=j.joint_3 / 1000.0,
            j4=j.joint_4 / 1000.0, j5=j.joint_5 / 1000.0, j6=j.joint_6 / 1000.0,
        )

    def _joints_close(self, target: JointAngles, tol_deg: float = None) -> bool:
        """True, если текущие углы суставов совпадают с target в пределах tol_deg."""
        if tol_deg is None:
            tol_deg = self.MOVE_SETTLE_TOL_DEG
        cur = self.get_joint_angles_deg()
        if cur is None:  # dry_run -- нечего ждать
            return True
        return all(
            abs(getattr(cur, name) - getattr(target, name)) <= tol_deg
            for name in ("j1", "j2", "j3", "j4", "j5", "j6")
        )

    def move_to_pose(self, pose: EndPose) -> None:
        if not self._enabled:
            raise RuntimeError("PiPER не включён: вызови enable() перед движением")
        self.enable_end_pose_mode()

        X, Y, Z, RX, RY, RZ = pose.as_milli()
        if self.dry_run:
            logger.info("[PiPER] (dry_run) EndPoseCtrl(x=%.1f, y=%.1f, z=%.1f, rx=%.1f, ry=%.1f, rz=%.1f)",
                        pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz)
            return
        try:
            self._piper.EndPoseCtrl(X, Y, Z, RX, RY, RZ)
        except Exception:
            logger.exception("[PiPER] EndPoseCtrl(...) попытка 1/2 не удалась -- повторяю")
            try:
                self._piper.EndPoseCtrl(X, Y, Z, RX, RY, RZ)
            except Exception:
                logger.exception("[PiPER] EndPoseCtrl(...) не удался -- команда НЕ ушла в CAN")
                raise

    def begin_jog(self, base_pose: Optional[EndPose] = None) -> EndPose:
        """
        Входим в режим ручной донастройки: фиксируем "базовую" позу (обычно
        текущая approach-поза руки), сбрасываем накопленное смещение в ноль
        и переключаемся в MOVE L. Дальше вызывай apply_jog() на каждый тик
        джойстика.
        """
        if base_pose is not None:
            self._jog_base = base_pose
        elif not self.dry_run:
            self._jog_base = self.get_end_pose_mm_deg()
        else:
            # dry_run без реального фидбека -- нулевая заглушка для демонстрации логики
            self._jog_base = EndPose()

        self._jog_offset = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.enable_end_pose_mode()
        logger.info("[PiPER] begin_jog: база = %s", self._jog_base)
        return self._jog_base

    def apply_jog(self, dx_mm: float, dy_mm: float, dz_mm: float) -> EndPose:
        """
        Прибавляет смещение (мм за тик) к накопленному офсету, клиппует его
        в безопасную коробку JOG_XY_LIMIT_MM/JOG_Z_LIMIT_MM вокруг базовой
        позы и отправляет получившуюся целевую позу через MOVE L.
        Ориентацию (rx/ry/rz) во время джога не трогаем -- только сдвиг.
        """
        if self._jog_base is None:
            raise RuntimeError("Сначала вызови begin_jog()")

        self._jog_offset["x"] = max(-self.JOG_XY_LIMIT_MM, min(self.JOG_XY_LIMIT_MM,
                                     self._jog_offset["x"] + dx_mm))
        self._jog_offset["y"] = max(-self.JOG_XY_LIMIT_MM, min(self.JOG_XY_LIMIT_MM,
                                     self._jog_offset["y"] + dy_mm))
        self._jog_offset["z"] = max(-self.JOG_Z_LIMIT_MM, min(self.JOG_Z_LIMIT_MM,
                                     self._jog_offset["z"] + dz_mm))

        target = EndPose(
            x=self._jog_base.x + self._jog_offset["x"],
            y=self._jog_base.y + self._jog_offset["y"],
            z=self._jog_base.z + self._jog_offset["z"],
            rx=self._jog_base.rx, ry=self._jog_base.ry, rz=self._jog_base.rz,
        )

        # Периодически (не на каждый тик, чтобы не тормозить джог) сверяем
        # РЕАЛЬНЫЙ режим руки по фидбеку, а не просто доверяем закэшированному
        # self._move_mode. Если режим где-то незаметно "уехал" (например, был
        # кратковременный сбой связи и рука откатилась в standby) -- обычный
        # move_to_pose() этого не заметит и будет молча слать EndPoseCtrl в
        # пустоту, а джойстик/VRA визуально перестанут действовать. Это
        # наиболее вероятная причина периодической "неотзывчивости" джога.
        if not self.dry_run:
            now = time.time()
            if now - self._last_mode_check >= self.JOG_MODE_RECHECK_S:
                self._last_mode_check = now
                if not self._ctrl_mode_feedback_is(0x01, 0x02):
                    logger.warning("[PiPER] джог: реальный режим руки разошёлся с ожидаемым MOVE L "
                                    "-- переотправляю ModeCtrl и повторяю попытку")
                    self._move_mode = None
                    self.enable_end_pose_mode()

        self.move_to_pose(target)
        return target

    def grasp_and_lift(self, lift_mm: float = 80.0,
                        gripper_close_mm: float = 15.0, gripper_effort_nm: float = 2.0,
                        gripper_open_mm: float = 60.0, step_delay_s: float = 1.2) -> None:
        """
        Закрывает захват ПРЯМО ИЗ ТЕКУЩЕЙ (уже подогнанной джойстиком) позы --
        без дополнительного автоматического спуска -- и поднимает на lift_mm.

        Почему без автоспуска: во время JOGGING высота (Z) уже находится под
        ручным управлением через VRA (см. tick_jog()/apply_jog() в
        pult_pickup_teleop.py). Если ПОСЛЕ того как оператор джойстиком/VRA
        уже подвёл захват к самому предмету, ещё и автоматически опускать
        руку на фиксированные descend_mm (как делает descend_grasp_lift()) --
        рука проедет НИЖЕ, чем нужно, к столу/предмету. Используй этот метод,
        если донастройка по Z уже приводит в позу непосредственно захвата.
        Если же donastройка останавливается ВЫШЕ предмета (approach-высота) и
        нужен ещё отдельный автоматический спуск перед хватом -- используй
        descend_grasp_lift() вместо этого метода.
        """
        if self._jog_base is None:
            raise RuntimeError("Сначала begin_jog()/apply_jog(), чтобы знать текущую позу")

        current = EndPose(
            x=self._jog_base.x + self._jog_offset["x"],
            y=self._jog_base.y + self._jog_offset["y"],
            z=self._jog_base.z + self._jog_offset["z"],
            rx=self._jog_base.rx, ry=self._jog_base.ry, rz=self._jog_base.rz,
        )

        logger.info("[PiPER] === grasp_and_lift: старт (хват из текущей позы, без автоспуска) ===")
        self.gripper(opening_mm=gripper_close_mm, effort_nm=gripper_effort_nm)
        time.sleep(0.5)

        up = EndPose(current.x, current.y, current.z + lift_mm, current.rx, current.ry, current.rz)
        self.move_to_pose(up)
        time.sleep(step_delay_s)
        logger.info("[PiPER] === grasp_and_lift: готово, объект захвачен ===")

    def descend_grasp_lift(self, descend_mm: float = 60.0, lift_mm: float = 80.0,
                            gripper_close_mm: float = 15.0, gripper_open_mm: float = 60.0,
                            step_delay_s: float = 1.2) -> None:
        """
        Из текущей (уже подогнанной джойстиком) позы: опуститься на
        descend_mm вниз, закрыть захват, подняться на lift_mm вверх.
        Вызывай после begin_jog()+несколько apply_jog().

        ПРИМЕЧАНИЕ: если во время JOGGING оператор уже сам сводит Z вплотную
        к предмету через VRA, этот метод даст ЛИШНИЙ спуск на descend_mm вниз
        (см. предупреждение в grasp_and_lift() выше) -- в pult_pickup_teleop.py
        по умолчанию используется grasp_and_lift(), а не этот метод.
        """
        if self._jog_base is None:
            raise RuntimeError("Сначала begin_jog()/apply_jog(), чтобы знать текущую позу")

        current = EndPose(
            x=self._jog_base.x + self._jog_offset["x"],
            y=self._jog_base.y + self._jog_offset["y"],
            z=self._jog_base.z + self._jog_offset["z"],
            rx=self._jog_base.rx, ry=self._jog_base.ry, rz=self._jog_base.rz,
        )

        logger.info("[PiPER] === descend_grasp_lift: старт ===")
        self.gripper(opening_mm=gripper_open_mm)
        time.sleep(0.3)

        down = EndPose(current.x, current.y, current.z - descend_mm, current.rx, current.ry, current.rz)
        self.move_to_pose(down)
        time.sleep(step_delay_s)

        self.gripper(opening_mm=gripper_close_mm, effort_nm=2.0)
        time.sleep(0.5)

        up = EndPose(down.x, down.y, down.z + lift_mm, down.rx, down.ry, down.rz)
        self.move_to_pose(up)
        time.sleep(step_delay_s)
        logger.info("[PiPER] === descend_grasp_lift: готово, объект захвачен ===")

    # -- фиксированный (без джога) сценарий на заранее откалиброванных углах --

    def pickup_sequence(self,
                         approach: JointAngles,
                         grasp: JointAngles,
                         lift: JointAngles,
                         gripper_open_mm: float = 60.0,
                         gripper_close_mm: float = 15.0,
                         step_delay_s: float = 1.5) -> None:
        """
        Старый (без ручной донастройки) сценарий на трёх заранее
        откалиброванных joint-позах. Работает надёжно, только если база
        каждый раз подъезжает в одно и то же место под одним и тем же углом.
        Если подъезд плавает -- используй begin_jog()/apply_jog()/
        descend_grasp_lift() вместо этого метода (см. pult_pickup_teleop.py).
        """
        if not self._enabled:
            raise RuntimeError("PiPER не включён: вызови enable() перед движением")

        logger.info("[PiPER] === pickup_sequence: старт ===")
        self.gripper(opening_mm=gripper_open_mm)
        time.sleep(0.3)

        self.move_joints(approach)
        time.sleep(step_delay_s)

        self.move_joints(grasp)
        time.sleep(step_delay_s)

        self.gripper(opening_mm=gripper_close_mm, effort_nm=2.0)
        time.sleep(0.5)

        self.move_joints(lift)
        time.sleep(step_delay_s)
        logger.info("[PiPER] === pickup_sequence: готово, объект захвачен ===")

    def release_and_home(self, gripper_open_mm: float = 60.0) -> None:
        """Отпустить предмет и вернуть руку в home."""
        if not self._enabled:
            raise RuntimeError("PiPER не включён: вызови enable() перед движением")
        logger.info("[PiPER] === release_and_home ===")
        self.gripper(opening_mm=gripper_open_mm)
        time.sleep(0.3)
        self._jog_base = None
        self.move_home()

    GRIPPER_MAX_MM = 70.0
    GRIPPER_MAX_EFFORT_NM = 5.0

    def gripper(self, opening_mm: float, effort_nm: float = 1.0, enable: bool = True) -> None:
        opening_mm = max(0.0, min(self.GRIPPER_MAX_MM, opening_mm))
        effort_nm = max(0.0, min(self.GRIPPER_MAX_EFFORT_NM, effort_nm))
        angle = int(round(opening_mm * 1000))
        effort = int(round(effort_nm * 1000))
        code = 0x01 if enable else 0x00

        if self.dry_run:
            logger.info("[PiPER] (dry_run) GripperCtrl(angle=%d, effort=%d, code=%s)",
                        angle, effort, hex(code))
            return
        try:
            self._piper.GripperCtrl(gripper_angle=angle, gripper_effort=effort, gripper_code=code)
        except Exception:
            logger.exception("[PiPER] GripperCtrl(...) попытка 1/2 не удалась -- повторяю")
            try:
                self._piper.GripperCtrl(gripper_angle=angle, gripper_effort=effort, gripper_code=code)
            except Exception:
                logger.exception("[PiPER] GripperCtrl(...) не удался -- команда НЕ ушла в CAN")
                raise

    def emergency_stop(self) -> None:
        if self.dry_run:
            logger.info("[PiPER] (dry_run) EmergencyStop(0x01)")
            return
        try:
            self._piper.EmergencyStop(emergency_stop=0x01)
        except Exception:
            logger.exception("[PiPER] EmergencyStop() не удался")
        self._enabled = False

    def get_joint_state(self) -> Optional[dict]:
        if self.dry_run:
            return None
        msg = self._piper.GetArmJointMsgs()
        return msg


# --------------------------------------------------------------------------- #
# Объединённая платформа
# --------------------------------------------------------------------------- #

class AgileXPlatform:
    def __init__(self,
                 bunker_can: str = "can_bunker",
                 piper_can: str = "can_piper",
                 dry_run: bool = True):
        self.dry_run = dry_run
        self.base = BunkerMiniController(can_name=bunker_can, dry_run=dry_run)
        self.arm = PiperArmController(can_name=piper_can, dry_run=dry_run)

    def startup(self) -> None:
        logger.info("=== AgileXPlatform startup ===")
        self.arm.connect()
        self.arm.enable()
        self.arm.move_home()
        self.base.enable()

    def shutdown(self) -> None:
        logger.info("=== AgileXPlatform shutdown ===")
        self.base.stop()
        self.arm.move_home()
        time.sleep(0.5)
        self.arm.disable()

    def drive(self, linear_mps: float, angular_rps: float, duration_s: float) -> None:
        self.base.set_velocity(linear_mps, angular_rps)
        time.sleep(duration_s)
        self.base.stop()

    def emergency_stop_all(self) -> None:
        logger.warning("!!! EMERGENCY STOP !!!")
        self.base.stop()
        self.arm.emergency_stop()


if __name__ == "__main__":
    platform = AgileXPlatform(dry_run=True)

    platform.startup()
    platform.arm.move_joints(JointAngles(j1=10, j2=20, j3=-30, j4=0, j5=15, j6=0))
    platform.arm.gripper(opening_mm=40)
    platform.arm.move_home()
    platform.drive(linear_mps=0.2, angular_rps=0.0, duration_s=1.0)
    platform.shutdown()
