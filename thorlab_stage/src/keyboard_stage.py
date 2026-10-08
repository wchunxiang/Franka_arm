#!/usr/bin/env python3
"""Keyboard control for up to 3 Thorlabs LTS150 linear stages (no ROS).

Run in a terminal (root is needed for USB/FTDI access, see README.md):

    cd thorlab_stage/src
    sudo python3 keyboard_stage.py                 # all 3 stages
    sudo python3 keyboard_stage.py --stages 1 3    # only connect stages 1 and 3
    python3 keyboard_stage.py --dry-run            # fake stages, no hardware
    python3 keyboard_stage.py --show-keys          # show what your terminal sends

Keys (press '?' in the program to see them again):

    arrow         -> stage 1        Up/Down    : big step forward/backward
    Shift+arrow   -> stage 2        Right/Left : small step forward/backward
    Alt+arrow     -> stage 3
    1 2 3         -> enable/disable stage 1/2/3
    h             -> home the enabled stages
    p             -> print positions
    q             -> quit

Stage 1/2/3 are the X/Y/Z stages of tele_xy_thorlab_v5.LinearStage.
"""
import argparse
import os
import re
import select
import sys
import termios
import tty
from collections import deque
from threading import Thread

#--------------config info.-------------------
step_stage_small = 3  # mm, same as thorlab_stage.py
step_stage_big = 10   # mm

MAX_DIST = 150  # mm, travel of the LTS150 (overridden by tele_xy_thorlab_v5.MAX_DIST)
HOME_VEL = 25   # mm/s, same as LinearStage.goHome()

STAGE_NAMES = ('X', 'Y', 'Z')  # stage 1, 2, 3

# +1/-1 per stage: which way "forward" moves the stage. Same signs as the
# [-dx, dy, -dz] used by thorlab_stage.py, so the motion matches the joystick node.
DIRECTION = (-1, +1, -1)

# modifier code sent with an arrow key (xterm: 1 none, 2 Shift, 3 Alt, 5 Ctrl)
# -> index of the stage it controls. If your terminal swallows Alt+arrow,
# run with --show-keys to see what it sends and add it here, e.g. 5: 2 for Ctrl.
MODIFIER_TO_STAGE = {1: 0, 2: 1, 3: 2}

# arrow key final letter -> (forward=+1/backward=-1, step size name)
ARROW_ACTION = {
    'A': (+1, 'big'),    # Up
    'B': (-1, 'big'),    # Down
    'C': (+1, 'small'),  # Right
    'D': (-1, 'small'),  # Left
}

HELP = """
  Arrow         stage 1 ({n0})      Up/Down    : big step forward/backward
  Shift+arrow   stage 2 ({n1})      Right/Left : small step forward/backward
  Alt+arrow     stage 3 ({n2})
  1 2 3         enable/disable stage 1/2/3
  h             home the enabled stages (asks first)
  p             print positions
  ?             this help
  q             quit
""".format(n0=STAGE_NAMES[0], n1=STAGE_NAMES[1], n2=STAGE_NAMES[2])


#--------------key parsing-------------------
# ESC [ A | ESC [ 1 ; <mod> A | ESC O A | ESC O <mod> A | ESC ESC [ A (Alt as prefix)
_ARROW = re.compile(rb'\x1b(\x1b)?(?:\[(?:1;(\d+))?|O(\d)?)([ABCD])')
# an escape sequence that is cut off at the end of the buffer
_PARTIAL = re.compile(rb'\x1b(?:\x1b|\x1b?\[[0-9;]*|\x1b?O\d?)?\Z')
# any other escape sequence (Home, PageUp, F-keys...): skipped, so its trailing
# characters (e.g. the '1' of ESC [ 1 ~) are not taken as key presses
_CSI = re.compile(rb'\x1b\[[0-9;]*[@-~]')
_SS3 = re.compile(rb'\x1bO[\x20-\x7e]')


def parse_keys(buf):
    """Split raw terminal bytes into events.

    Returns (events, rest). An event is ('arrow', modifier, letter) or
    ('key', char); rest is an unfinished escape sequence to prepend to the next read.
    """
    events = []
    i = 0
    while i < len(buf):
        if buf[i] != 0x1b:
            events.append(('key', chr(buf[i])))
            i += 1
            continue
        m = _ARROW.match(buf, i)
        if m:
            modifier = int(m.group(2) or m.group(3) or (3 if m.group(1) else 1))
            events.append(('arrow', modifier, m.group(4).decode()))
            i = m.end()
            continue
        if _PARTIAL.match(buf, i):
            return events, buf[i:]
        m = _CSI.match(buf, i) or _SS3.match(buf, i)
        if m:
            i = m.end()
        else:  # lone ESC, or Alt+<key>: ignore
            i += 2 if i + 1 < len(buf) else 1
    return events, b''


class KeyReader:
    def __init__(self, fd):
        self.fd = fd
        self.pending = b''
        self.queue = deque()

    def get(self):
        """Block until the next event; None when stdin is closed."""
        while not self.queue:
            timeout = 0.05 if self.pending else None
            ready, _, _ = select.select([self.fd], [], [], timeout)
            if not ready:  # unfinished escape sequence never completed: drop it
                self.pending = b''
                continue
            data = os.read(self.fd, 64)
            if not data:
                return None
            events, self.pending = parse_keys(self.pending + data)
            self.queue.extend(events)
        return self.queue.popleft()

    def flush(self):
        """Drop key presses made while the stage was moving (e.g. key repeat)."""
        self.queue.clear()
        self.pending = b''
        termios.tcflush(self.fd, termios.TCIFLUSH)


#--------------stages-------------------
class FakeAxis:
    """Stand-in for pyAPT.LTS150 used by --dry-run."""
    def __init__(self, pos=MAX_DIST / 2):
        self.pos = pos

    def position(self):
        return self.pos

    def goto(self, pos):
        self.pos = pos

    def home(self, velocity=None):
        self.pos = 0


def connect_stages(stage_ids, dry_run):
    """Return (axes, max_dist). axes[i] is None for a stage that is not connected."""
    if dry_run:
        return [FakeAxis() if i in stage_ids else None for i in range(3)], MAX_DIST

    import tele_xy_thorlab_v5 as lts  # imports pyAPT -> needs pylibftdi
    avail = tuple(i in stage_ids for i in range(3))
    stage = lts.LinearStage(avail)  # opens the enabled stages and sets velocity
    axes = [getattr(stage, attr) if avail[i] else None
            for i, attr in enumerate(('x_stage', 'y_stage', 'z_stage'))]
    return axes, lts.MAX_DIST


class KeyboardStage:
    def __init__(self, axes, max_dist, step_small, step_big, reader):
        self.axes = axes
        self.max_dist = max_dist
        self.steps = {'small': step_small, 'big': step_big}
        self.reader = reader
        self.enabled = [a is not None for a in axes]

    def label(self, i):
        return 'stage {} ({})'.format(i + 1, STAGE_NAMES[i])

    def status(self):
        parts = []
        for i, axis in enumerate(self.axes):
            if axis is None:
                parts.append('{} --'.format(STAGE_NAMES[i]))
            elif not self.enabled[i]:
                parts.append('{} off'.format(STAGE_NAMES[i]))
            else:
                parts.append('{} {:.2f}'.format(STAGE_NAMES[i], axis.position()))
        return '[' + ' | '.join(parts) + ' mm]'

    def move(self, i, forward, size):
        if self.axes[i] is None:
            print('{} is not connected (start with --stages)'.format(self.label(i)))
            return
        if not self.enabled[i]:
            print('{} is disabled (press {} to enable)'.format(self.label(i), i + 1))
            return
        delta = forward * DIRECTION[i] * self.steps[size]
        try:
            cur = self.axes[i].position()
            target = cur + delta
            if target <= 0 or target >= self.max_dist:
                print('!!!----{}: reach {} limit----!!!'.format(
                    STAGE_NAMES[i], '0' if target <= 0 else 'max'))
            target = min(max(target, 0), self.max_dist)
            self.axes[i].goto(target)
            print('{} {} {}: {:+.1f} mm  {}'.format(
                self.label(i), size, 'forward' if forward > 0 else 'backward',
                target - cur, self.status()))
        except Exception as e:
            print('{} move failed: {}'.format(self.label(i), e))
        self.reader.flush()

    def toggle(self, i):
        if self.axes[i] is None:
            print('{} is not connected (start with --stages)'.format(self.label(i)))
            return
        self.enabled[i] = not self.enabled[i]
        print('{} {}  {}'.format(self.label(i),
                                 'enabled' if self.enabled[i] else 'disabled', self.status()))

    def home(self):
        ids = [i for i in range(3) if self.axes[i] is not None and self.enabled[i]]
        if not ids:
            print('no enabled stage to home')
            return
        print('Home {}? They will move to 0 mm. [y/N] '.format(
            ', '.join(self.label(i) for i in ids)), end='', flush=True)
        ev = self.reader.get()
        print()
        if ev != ('key', 'y'):
            print('home cancelled')
            return

        def run(i):
            try:
                self.axes[i].home(velocity=HOME_VEL)
            except Exception as e:
                print('{} home failed: {}'.format(self.label(i), e))

        threads = [Thread(target=run, args=(i,)) for i in ids]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        print('homed  ' + self.status())
        self.reader.flush()

    def handle(self, ev):
        """Returns False when the user wants to quit."""
        if ev[0] == 'arrow':
            _, modifier, letter = ev
            if modifier in MODIFIER_TO_STAGE:
                forward, size = ARROW_ACTION[letter]
                self.move(MODIFIER_TO_STAGE[modifier], forward, size)
            return True
        key = ev[1]
        if key == 'q':
            return False
        if key in '123':
            self.toggle(int(key) - 1)
        elif key == 'h':
            self.home()
        elif key == 'p':
            print(self.status())
        elif key == '?':
            print(HELP)
        return True


def show_keys(fd):
    """Print what the terminal sends for each key (to find the Alt+arrow code)."""
    print('Press keys to see their bytes; q quits.')
    while True:
        select.select([fd], [], [])
        data = os.read(fd, 64)
        events, _ = parse_keys(data)
        print('{!r:<24} -> {}'.format(data, events))
        if data == b'q':
            return


def main():
    parser = argparse.ArgumentParser(description='Control up to 3 Thorlabs stages with the keyboard.')
    parser.add_argument('--stages', type=int, nargs='+', choices=(1, 2, 3), default=[1, 2, 3],
                        help='stages to connect/enable (default: 1 2 3)')
    parser.add_argument('--small', type=float, default=step_stage_small, help='small step in mm')
    parser.add_argument('--big', type=float, default=step_stage_big, help='big step in mm')
    parser.add_argument('--dry-run', action='store_true', help='use fake stages (no hardware)')
    parser.add_argument('--show-keys', action='store_true', help='print the bytes of each key press')
    args = parser.parse_args()
    if args.small <= 0 or args.big <= 0:
        parser.error('step sizes must be positive')
    if not sys.stdin.isatty():
        sys.exit('keyboard_stage.py needs an interactive terminal')

    fd = sys.stdin.fileno()
    if args.show_keys:
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            show_keys(fd)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return

    stage_ids = sorted(set(a - 1 for a in args.stages))
    axes, max_dist = connect_stages(stage_ids, args.dry_run)

    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        reader = KeyReader(fd)
        controller = KeyboardStage(axes, max_dist, args.small, args.big, reader)
        print(HELP)
        print('small step {} mm, big step {} mm{}'.format(
            args.small, args.big, '  [DRY RUN]' if args.dry_run else ''))
        print(controller.status())
        while True:
            ev = reader.get()
            if ev is None or not controller.handle(ev):
                break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    print('bye')


if __name__ == '__main__':
    main()
