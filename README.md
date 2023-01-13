# Humanoid Robot Wrestling Controller Example

[![webots.cloud - Competition](https://img.shields.io/badge/webots.cloud-Competition-007ACC)][1]

## David controller

Demonstrates how to use a sensor, here an accelerometer to detect a fall. We use the Accelerometer class from the [sensor.py](./controllers/utils/sensors.py) module to have a rolling average on the accelerometer's readings.

Depending on the fall direction, the robot will play a different motion, which is implemented by a simple Finite State Machine using the Finite_state_machine class from the [fsm.py](./controllers/utils/fsm.py) module.

Beats [Charlie](https://github.com/cyberbotics/wrestling-charlie) by getting up after being knocked down.

Here is the [participant.py](./controllers/participant/participant.py) file:

``` Python
import sys
from controller import Robot
sys.path.append('..')
from utils.accelerometer import Accelerometer
from utils.finite_state_machine import FiniteStateMachine
from utils.motion_library import MotionLibrary
from utils.current_motion_manager import CurrentMotionManager


class David (Robot):
    def __init__(self):
        Robot.__init__(self)

        # retrieves the WorldInfo.basicTimeTime (ms) from the world file
        self.time_step = int(self.getBasicTimeStep())
        # the Finite State Machine (FSM) is a way of representing a robot's behavior as a sequence of states
        self.fsm = FiniteStateMachine(
            states=['DEFAULT', 'BLOCKING_MOTION', 'FRONT_FALL', 'BACK_FALL'],
            initial_state='DEFAULT',
            actions={
                'BLOCKING_MOTION': self.pending,
                'DEFAULT': self.walk,
                'FRONT_FALL': self.front_fall,
                'BACK_FALL': self.back_fall
            }
        )

        # accelerometer
        self.accelerometer = Accelerometer(
            self.getDevice('accelerometer'), self.time_step)

        self.leds = {
            'right': self.getDevice('Face/Led/Right'),
            'left':  self.getDevice('Face/Led/Left')
        }

        # Shoulder roll motors for getting up from a side fall
        self.RShoulderRoll = self.getDevice('RShoulderRoll')
        self.LShoulderRoll = self.getDevice('LShoulderRoll')

        # load motion files
        self.current_motion = CurrentMotionManager()
        self.library = MotionLibrary()

    def run(self):
        self.leds['right'].set(0x0000ff)
        self.leds['left'].set(0x0000ff)
        self.current_motion.set(self.library.get('Stand'))
        self.fsm.transition_to('BLOCKING_MOTION')

        while self.step(self.time_step) != -1:
            t = self.getTime()
            self.detect_fall()
            self.fsm.execute_action()

    def detect_fall(self):
        """Detect a fall and update the FSM state."""
        [acc_x, acc_y, _] = self.accelerometer.get_new_average()
        if acc_x < -7:
            self.fsm.transition_to('FRONT_FALL')
        elif acc_x > 7:
            self.fsm.transition_to('BACK_FALL')
        if acc_y < -7:
            # Fell to its right, pushing itself on its back
            self.RShoulderRoll.setPosition(-1.2)
        elif acc_y > 7:
            # Fell to its left, pushing itself on its back
            self.LShoulderRoll.setPosition(1.2)

    def pending(self):
        # waits for the current motion to finish before doing anything else
        if self.current_motion.is_over():
            self.fsm.transition_to('DEFAULT')

    def walk(self):
        if self.current_motion.get() != self.library.get('ForwardLoop'):
            self.current_motion.set(self.library.get('ForwardLoop'))

    def front_fall(self):
        self.current_motion.set(self.library.get('GetUpFront'))
        self.fsm.transition_to('BLOCKING_MOTION')

    def back_fall(self):
        self.current_motion.set(self.library.get('GetUpBack'))
        self.fsm.transition_to('BLOCKING_MOTION')


# create the Robot instance and run main loop
wrestler = David()
wrestler.run()
```

[Eve](https://github.com/cyberbotics/wrestling-eve) is a more advanced robot controller able to win against David.

[1]: https://webots.cloud/run?version=R2022b&url=https%3A%2F%2Fgithub.com%2Fcyberbotics%2Fwrestling%2Fblob%2Fmain%2Fworlds%2Fwrestling.wbt&type=competition "Leaderboard"
