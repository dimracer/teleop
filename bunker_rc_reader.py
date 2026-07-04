#!/usr/bin/env python3
# coding=utf-8
"""
bunker_rc_reader.py

Chtenie sostoyaniya shtatnogo pulta FS (idet v komplekte s Bunker Mini 2.0)
napryamuyu s CAN-shiny, v obhod pyagxrobots.

Pochemu napryamuyu, a ne cherez pyagxrobots:
    V pyagxrobots README klass `RcStateMessage` zayavlen, no v ishodnikah
    (pyagxrobots/pysdkugv.py, agxbase.py) u BunkerBase gettery dlya RC
    (SWA/SWB/.../stiki) ne realizovany -- biblioteka ih ne otdaet.
    Poetomu chitaem kadr CAN ID 0x241 sami, po ofitsialnomu protokolu iz
    "BUNKER MINI 2.0 User Manual V2 2023.01", razdel 3.3.1, tablitsa 3.11
    "Remote control information feedback".

VAZHNO PRO SOVMESHCHENIE PULTA I CAN:
    Kadr 0x241 -- eto PASSIVNAYA TELEMETRIYA. Shassi shlet ego kazhdye 20 ms
    nezavisimo ot togo, v kakom rezhime nahoditsya SWB (remote-control ili
    command-control). Slushat etot kadr mozhno, voobsche ne vklyuchaya
    CAN-command-rezhim (0x421) -- to est pult kak rulil shassi napryamuyu,
    tak i prodolzhaet rulit, a host prosto "podslushivaet" ego sostoyanie.
    Eto klyuchevaya ideya: fizicheskoe vozhdenie ostaetsya polnostyu na pulte,
    host tolko reagiruet na pereklyuchateli.

SVOBODNYE KANALY PULTA (po manualu):
    - SWA i SWD -- "temporarily disabled", shassi ih ne ispolzuet voobsche.
    - VRA (levaya krutilka) -- tozhe ne ispolzuetsya logikoy shassi Bunker Mini.
    Itogo est 2 svobodnyh 2-pozitsionnyh pereklyuchatelya + 1 krutilka
    (analogovaya, -100..100), kotorye mozhno otdat pod upravlenie PiPER,
    ne trogaya shtatnoe vozhdenie (S1/S2/SWB/SWC).

    Eto NE polnotsennoe ruchnoe teleupravlenie rukoy (6 sustavov na 3
    svobodnyh kanala ne razmotat vmenyaemo) -- eto triggery/diskretnye
    komandy: "vypolnit stsenariy zahvata", "otkryt/zakryt zahvat",
    "vernut ruku v home" i t.p. Dlya polnogo ruchnogo teleupravleniya rukoy
    (dzhoystikom po kazhdomu sustavu) nuzhen otdelnyy kontroller
    (gejmpad/dzhoystik na hoste ili shtatnyy master-arm dlya PiPER) --
    mozhno sdelat otdelno, esli ponadobitsya.

PRO PRIORITET UPRAVLENIYA BAZOY (razdel 3.3.2 manuala):
    Poka pult vklyuchen, on imeet naivysshiy prioritet i blokiruet lyubye
    CAN-komandy dvizheniya (0x111) ot kompyutera. Znachit, poka edem na
    pulte, nash kod NE upravlyaet bazoy i ne mozhet ee "siloy" ostanovit --
    tolko chitaet. Dlya stsenariya "podekhal -> vzyal" eto i ne trebuetsya:
    podezd ostaetsya polnostyu v rukah operatora. No pered tem kak dat
    ruke nachat zahvat, stoit proverit, chto baza fizicheski stoit -- sm.
    is_stationary() nizhe, edinstvennyy nadezhnyy sposob ubeditsya v etom
    programmno (po fakt. skorosti iz kadra 0x221, Table 3.3 manuala).

Format kadra 0x241 (Motorola/big-endian byte order, kak ves protokol Bunker):
    byte0: bit[0:2]=SWA (2=up,3=down), bit[2:4]=SWB (2=up,1=mid,3=down),
           bit[4:6]=SWC (2=up,1=mid,3=down), bit[6:8]=SWD (2=up,3=down)
    byte1: pravyy stik vlevo/vpravo   int8 [-100,100]
    byte2: pravyy stik vverh/vniz     int8 [-100,100]
    byte3: levyy stik vverh/vniz      int8 [-100,100]   (S1, gaz)
    byte4: levyy stik vlevo/vpravo    int8 [-100,100]   (S2, povorot)
    byte5: levaya krutilka VRA        int8 [-100,100]
    byte6: rezerv
    byte7: schetchik 0..255

Format kadra 0x221 (Motion control feedback, Table 3.3 manuala):
    byte0-1: lineynaya skorost, signed int16 big-endian, x1000 -> 0.001 m/s
    byte2-3: uglovaya skorost,  signed int16 big-endian, x100  -> 0.01 rad/s

Zavisimosti (NE ustanavlivalis, stav vruchnuyu po soglasovaniyu):
    pip3 install python-can
"""

from __future__ import annotations

import time
import logging
from dataclasses import dataclass
from typing import Callable, Optional

logger = logging.getLogger("bunker_rc_reader")

RC_FEEDBACK_CAN_ID = 0x241
MOTION_FEEDBACK_CAN_ID = 0x221


@dataclass
class RcState:
    swa: int
    swb: int
    swc: int
    swd: int
    right_lr: int
    right_ud: int
    left_ud: int
    left_lr: int
    vra: int
    count: int
    timestamp: float

    @classmethod
    def from_can_data(cls, data: bytes, timestamp: float) -> "RcState":
        if len(data) < 8:
            raise ValueError(f"Ozhidalos 8 bayt v kadre 0x241, poluceno {len(data)}")

        b0 = data[0]
        swa = (b0 >> 0) & 0b11
        swb = (b0 >> 2) & 0b11
        swc = (b0 >> 4) & 0b11
        swd = (b0 >> 6) & 0b11

        def to_signed8(x: int) -> int:
            return x - 256 if x > 127 else x

        return cls(
            swa=swa,
            swb=swb,
            swc=swc,
            swd=swd,
            right_lr=to_signed8(data[1]),
            right_ud=to_signed8(data[2]),
            left_ud=to_signed8(data[3]),
            left_lr=to_signed8(data[4]),
            vra=to_signed8(data[5]),
            count=data[7],
            timestamp=timestamp,
        )


class BunkerRcReader:
    """
    Slushaet CAN ID 0x241 (pult) i 0x221 (fakt. skorost) na interfeyse
    Bunker Mini i hranit poslednee sostoyanie + umeet zvat callback-i na
    perehody svobodnyh kanalov (SWA, SWD).

    Ispolzovanie:
        rc = BunkerRcReader(can_name="can_bunker")
        rc.on_swd_down(lambda state: piper_arm.pickup_sequence())
        rc.start()
        ...
        rc.stop()
    """

    def __init__(self, can_name: str = "can_bunker", dry_run: bool = True):
        self.can_name = can_name
        self.dry_run = dry_run
        self._bus = None
        self._running = False
        self.state: Optional[RcState] = None
        self.linear_mps: Optional[float] = None
        self.angular_radps: Optional[float] = None
        self._last_swa = None
        self._last_swd = None
        self._callbacks_swa_down = []
        self._callbacks_swd_down = []
        self._callbacks_any = []

        if not dry_run:
            import can  # python-can
            self._can = can
            self._bus = can.interface.Bus(channel=can_name, bustype="socketcan")
        else:
            logger.info("[RC] dry_run=True: real'nogo podklyucheniya k %s ne budet", can_name)

    def on_swa_down(self, callback: Callable[[RcState], None]) -> None:
        self._callbacks_swa_down.append(callback)

    def on_swd_down(self, callback: Callable[[RcState], None]) -> None:
        self._callbacks_swd_down.append(callback)

    def on_update(self, callback: Callable[[RcState], None]) -> None:
        self._callbacks_any.append(callback)

    def start(self) -> None:
        self._running = True
        logger.info("[RC] start proslushivaniya CAN ID 0x%X na %s", RC_FEEDBACK_CAN_ID, self.can_name)

    def stop(self) -> None:
        self._running = False
        if self._bus is not None:
            self._bus.shutdown()
        logger.info("[RC] ostanovleno")

    def poll_once(self, timeout: float = 0.1) -> Optional[RcState]:
        if self.dry_run:
            return None

        msg = self._bus.recv(timeout=timeout)
        if msg is None:
            return None

        if msg.arbitration_id == MOTION_FEEDBACK_CAN_ID and len(msg.data) >= 4:
            self._process_motion(bytes(msg.data))
            return None

        if msg.arbitration_id != RC_FEEDBACK_CAN_ID:
            return None

        state = RcState.from_can_data(bytes(msg.data), time.time())
        self._process_state(state)
        return state

    def _process_motion(self, data: bytes) -> None:
        linear_raw = int.from_bytes(data[0:2], byteorder="big", signed=True)
        angular_raw = int.from_bytes(data[2:4], byteorder="big", signed=True)
        self.linear_mps = linear_raw / 1000.0
        self.angular_radps = angular_raw / 100.0

    def is_stationary(self, lin_eps: float = 0.02, ang_eps: float = 0.05) -> bool:
        if self.linear_mps is None or self.angular_radps is None:
            return False
        return abs(self.linear_mps) < lin_eps and abs(self.angular_radps) < ang_eps

    def spin(self, poll_timeout: float = 0.1) -> None:
        self.start()
        try:
            while self._running:
                self.poll_once(timeout=poll_timeout)
        finally:
            self.stop()

    def _process_state(self, state: RcState) -> None:
        self.state = state
        for cb in self._callbacks_any:
            cb(state)

        if self._last_swd != 3 and state.swd == 3:
            for cb in self._callbacks_swd_down:
                cb(state)
        if self._last_swa != 3 and state.swa == 3:
            for cb in self._callbacks_swa_down:
                cb(state)

        self._last_swa = state.swa
        self._last_swd = state.swd


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

    rc = BunkerRcReader(can_name="can_bunker", dry_run=True)
    rc.on_swd_down(lambda s: logger.info("Trigger: SWD down -> zapustit stsenariy zahvata PiPER"))
    rc.on_swa_down(lambda s: logger.info("Trigger: SWA down -> vernut ruku v home / otpustit obekt"))

    logger.info("Eto dry-run demo: bez realnogo CAN prosto pokazyvaet formu API.")
    logger.info("Na realnom stende vyzovi rc.spin() v otdelnom potoke.")
