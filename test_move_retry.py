#!/usr/bin/env python3
# coding=utf-8
"""
Проверка логики надёжной доставки команд движения (agilex_platform.py):
  - пара ModeCtrl+JointCtrl перед каждой отправкой (канонный паттерн piper_sdk);
  - периодическая переотправка пары до подтверждения по фидбеку;
  - разворот-сброс turn_lower_release_home (порядок и цели шагов).
"""
import types
from agilex_platform import PiperArmController, JointAngles


class FakePiper:
    """Эмулирует piper_sdk. ignore_without_fresh_mode=True воспроизводит поведение
    прошивки со стенда: JointCtrl без свежего ModeCtrl перед ним игнорируется."""

    def __init__(self, ignore_without_fresh_mode: bool = False, lose_first_n: int = 0):
        self.ignore_without_fresh_mode = ignore_without_fresh_mode
        self.lose_first_n = lose_first_n
        self.jointctrl_calls = 0
        self.modectrl_calls = 0
        self.fresh_mode = False        # был ли ModeCtrl непосредственно перед командой
        self.pos = [0.0] * 6           # текущие углы, градусы
        self.target = None
        self.ctrl_mode = 0x01
        self.mode_feed = 0x01
        self.calls = []                # журнал вызовов для проверки порядка

    def ModeCtrl(self, ctrl_mode, move_mode, move_spd_rate_ctrl):
        self.modectrl_calls += 1
        self.ctrl_mode = ctrl_mode
        self.mode_feed = move_mode
        self.fresh_mode = True
        self.calls.append(("ModeCtrl", move_mode))

    def JointCtrl(self, *j_milli):
        self.jointctrl_calls += 1
        self.calls.append(("JointCtrl", tuple(j_milli)))
        if self.jointctrl_calls <= self.lose_first_n:
            self.fresh_mode = False
            return                       # кадр 'потерян'
        if self.ignore_without_fresh_mode and not self.fresh_mode:
            return                       # прошивка молча игнорирует без свежего ModeCtrl
        self.fresh_mode = False
        self.target = [v / 1000.0 for v in j_milli]

    def GripperCtrl(self, gripper_angle, gripper_effort, gripper_code):
        self.calls.append(("GripperCtrl", gripper_angle))

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
    arm.MOVE_RESEND_PERIOD_S = 0.15
    arm.MOVE_SETTLE_TIMEOUT_S = 0.8
    arm.MOVE_SETTLE_POLL_S = 0.02
    return arm


TARGET = JointAngles(j1=0, j2=40, j3=-60, j4=0, j5=20, j6=0)

# 1. Обычная доставка: пара ModeCtrl+JointCtrl, приход подтверждён
fake = FakePiper()
assert make_arm(fake).move_joints(TARGET, wait_settle=True) is True
assert fake.modectrl_calls >= 1 and fake.jointctrl_calls >= 1
print("OK 1: пара ModeCtrl+JointCtrl отправлена, settled=True")

# 2. Каждый JointCtrl предварён ModeCtrl (канонный паттерн SDK)
idx = {"ModeCtrl": [], "JointCtrl": []}
for i, (name, _) in enumerate(fake.calls):
    if name in idx:
        idx[name].append(i)
for jc in idx["JointCtrl"]:
    assert any(mc == jc - 1 for mc in idx["ModeCtrl"]), f"JointCtrl@{jc} без ModeCtrl перед ним"
print("OK 2: каждый JointCtrl идёт сразу после ModeCtrl")

# 3. Прошивка игнорирует JointCtrl без свежего ModeCtrl (симптом со стенда) --
#    с парой команда всё равно доставляется с первого вызова move_joints
fake = FakePiper(ignore_without_fresh_mode=True)
assert make_arm(fake).move_joints(TARGET, wait_settle=True) is True
print("OK 3: 'капризная' прошивка (нужен свежий ModeCtrl) -> доставлено с 1-го вызова")

# 4. Первые 2 кадра теряются -> переотправка пары по периоду добивает, True
fake = FakePiper(lose_first_n=2)
assert make_arm(fake).move_joints(TARGET, wait_settle=True) is True
assert fake.jointctrl_calls >= 3, fake.jointctrl_calls
print("OK 4: потеря первых кадров -> переотправка по периоду, settled=True")

# 5. Всё теряется -> False после всех попыток
fake = FakePiper(lose_first_n=10**6)
assert make_arm(fake).move_joints(TARGET, wait_settle=True) is False
print("OK 5: всё потеряно -> False после", PiperArmController.MOVE_SEND_RETRIES, "попыток")

# 6. Рука уже в целевой позе -> мгновенно True
fake = FakePiper(lose_first_n=10**6)
fake.pos = [0.0, 40.0, -60.0, 0.0, 20.0, 0.0]
assert make_arm(fake).move_joints(TARGET, wait_settle=True) is True
print("OK 6: уже в позе -> сразу True")

# 7. Разворот-сброс: порядок шагов и целевые углы
fake = FakePiper()
arm = make_arm(fake)
arm._jog_base = object()  # имитация 'после захвата'
fake.pos = [0.0, 40.0, -60.0, 0.0, 20.0, 0.0]
fake.target = list(fake.pos)
assert arm.turn_lower_release_home() is True

targets = [c[1] for c in fake.calls if c[0] == "JointCtrl"]
grip_idx = [i for i, c in enumerate(fake.calls) if c[0] == "GripperCtrl"]
uniq = []
for t in targets:
    if not uniq or uniq[-1] != t:
        uniq.append(t)
# шаг 1: разворот J1 -> 150°, J2/J3 как при захвате
assert uniq[0][0] == 150000 and uniq[0][1] == 40000 and uniq[0][2] == -60000, uniq[0]
# шаг 2: спуск J2/J3 на 40%: 40->56, -60->-84
assert uniq[1][0] == 150000 and uniq[1][1] == 56000 and uniq[1][2] == -84000, uniq[1]
# шаг 3: отпускание захвата ПОСЛЕ спуска и ДО ухода домой
last_lower_i = max(i for i, c in enumerate(fake.calls)
                   if c[0] == "JointCtrl" and c[1][1] == 56000)
home_i = min(i for i, c in enumerate(fake.calls)
             if c[0] == "JointCtrl" and c[1] == (0, 0, 0, 0, 0, 0))
assert any(last_lower_i < g < home_i for g in grip_idx), (last_lower_i, grip_idx, home_i)
# шаг 4: конечная цель -- zero point
assert uniq[-1] == (0, 0, 0, 0, 0, 0), uniq[-1]
assert arm._jog_base is None
print("OK 7: разворот J1=150°, спуск J2=56°/J3=-84°, отпустить, zero point -- порядок верный")

print("\nВсе 7 проверок пройдены.")
