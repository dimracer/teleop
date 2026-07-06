#!/usr/bin/env python3
# coding=utf-8
"""Проверка логики ретраев move_joints(wait_settle=True) на фейковом piper."""
import types
from agilex_platform import PiperArmController, JointAngles


class FakePiper:
    """Эмулирует piper_sdk: кадры JointCtrl можно 'терять' первые N раз."""

    def __init__(self, lose_first_n: int, mode_ok: bool = True):
        self.lose_first_n = lose_first_n
        self.jointctrl_calls = 0
        self.modectrl_calls = 0
        self.pos = [0.0] * 6      # текущие углы, градусы
        self.target = None
        self.ctrl_mode = 0x01 if mode_ok else 0x00
        self.mode_feed = 0x01 if mode_ok else 0x00

    def JointCtrl(self, *j_milli):
        self.jointctrl_calls += 1
        if self.jointctrl_calls <= self.lose_first_n:
            return  # кадр 'потерян': прошивка его не получила, исключения нет
        self.target = [v / 1000.0 for v in j_milli]

    def ModeCtrl(self, ctrl_mode, move_mode, move_spd_rate_ctrl):
        self.modectrl_calls += 1
        self.ctrl_mode = ctrl_mode
        self.mode_feed = move_mode

    def GetArmStatus(self):
        st = types.SimpleNamespace(ctrl_mode=self.ctrl_mode, mode_feed=self.mode_feed)
        return types.SimpleNamespace(arm_status=st)

    def GetArmJointMsgs(self):
        # эмуляция движения: за каждый опрос доезжаем 60% остатка до цели
        if self.target is not None:
            self.pos = [p + 0.6 * (t - p) for p, t in zip(self.pos, self.target)]
        js = types.SimpleNamespace(**{f"joint_{i+1}": int(self.pos[i] * 1000) for i in range(6)})
        return types.SimpleNamespace(joint_state=js)


def make_arm(fake: FakePiper) -> PiperArmController:
    arm = PiperArmController(dry_run=True)
    arm.dry_run = False           # включаем 'реальный' путь, но с фейковым piper
    arm._piper = fake
    arm._enabled = True
    arm._move_mode = "joint"
    # ускоряем тест
    arm.MOVE_NO_PROGRESS_TIMEOUT_S = 0.3
    arm.MOVE_SETTLE_TIMEOUT_S = 1.5
    arm.MOVE_SETTLE_POLL_S = 0.02
    return arm


TARGET = JointAngles(j1=0, j2=40, j3=-60, j4=0, j5=20, j6=0)

# 1. Кадр доходит с первого раза -> одна попытка, True
fake = FakePiper(lose_first_n=0)
assert make_arm(fake).move_joints(TARGET, wait_settle=True) is True
assert fake.jointctrl_calls == 1, fake.jointctrl_calls
print("OK 1: кадр дошёл сразу -> 1 отправка, settled=True")

# 2. Первый кадр потерян -> переотправка по no-progress, True со 2-й попытки
fake = FakePiper(lose_first_n=1)
assert make_arm(fake).move_joints(TARGET, wait_settle=True) is True
assert fake.jointctrl_calls == 2, fake.jointctrl_calls
print("OK 2: первый кадр потерян -> переотправлен, settled=True со 2-й попытки")

# 3. Первые два кадра потеряны -> True с 3-й попытки
fake = FakePiper(lose_first_n=2)
assert make_arm(fake).move_joints(TARGET, wait_settle=True) is True
assert fake.jointctrl_calls == 3, fake.jointctrl_calls
print("OK 3: два кадра потеряны -> settled=True с 3-й попытки")

# 4. Все кадры теряются -> False после MOVE_SEND_RETRIES попыток
fake = FakePiper(lose_first_n=99)
assert make_arm(fake).move_joints(TARGET, wait_settle=True) is False
assert fake.jointctrl_calls == PiperArmController.MOVE_SEND_RETRIES, fake.jointctrl_calls
print("OK 4: всё потеряно -> False после", fake.jointctrl_calls, "попыток")

# 5. Реальный режим руки разошёлся с кэшем -> ModeCtrl переотправляется сам
fake = FakePiper(lose_first_n=0, mode_ok=False)
arm = make_arm(fake)              # кэш говорит "joint", фидбек -- нет
assert arm.move_joints(TARGET, wait_settle=True) is True
assert fake.modectrl_calls >= 1, fake.modectrl_calls
print("OK 5: расхождение режима по фидбеку -> ModeCtrl переотправлен, settled=True")

# 6. Рука уже в целевой позе -> мгновенно True
fake = FakePiper(lose_first_n=99)
fake.pos = [0.0, 40.0, -60.0, 0.0, 20.0, 0.0]
assert make_arm(fake).move_joints(TARGET, wait_settle=True) is True
print("OK 6: уже в позе -> сразу True (без лишнего ожидания)")

print("\nВсе 6 проверок пройдены.")
