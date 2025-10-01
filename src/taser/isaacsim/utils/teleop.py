import carb
import numpy as np
import omni.appwindow


class Teleop:
    def __init__(self):
        # bindings for keyboard to command
        self._input_keyboard_mapping = {
            # forward command
            "NUMPAD_8": [1.0, 0.0, 0.0],
            "UP": [1.0, 0.0, 0.0],
            # back command
            "NUMPAD_2": [-1.0, 0.0, 0.0],
            "DOWN": [-1.0, 0.0, 0.0],
            # yaw command (negative)
            "NUMPAD_6": [0.0, 0.0, -1.0],
            "RIGHT": [0.0, 0.0, -1.0],
            # yaw command (positive)
            "NUMPAD_4": [0.0, 0.0, 1.0],
            "LEFT": [0.0, 0.0, 1.0],
        }

        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._sub_keyboard_event
        )

        self._command = np.array([[0.0, 0.0, 0.0]], dtype=np.float16)  # x, y, yaw

    def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
        """
        Keyboard subscriber callback to when kit is updated.

        """

        # when a key is pressed for released  the command is adjusted w.r.t the key-mapping
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            # on pressing, the command is incremented
            if event.input.name in self._input_keyboard_mapping:
                self._command += np.array(
                    self._input_keyboard_mapping[event.input.name]
                )

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            # on release, the command is decremented
            if event.input.name in self._input_keyboard_mapping:
                self._command -= np.array(
                    self._input_keyboard_mapping[event.input.name]
                )
        return True

    def get_command(self) -> np.ndarray:
        return self._command
