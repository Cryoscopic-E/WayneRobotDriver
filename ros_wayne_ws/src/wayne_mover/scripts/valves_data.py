from std_msgs.msg import UInt8MultiArray

class ValvesData:
    def __init__(self) -> None:
        self.states = UInt8MultiArray()
        self.states.layout.dim = 1
        self.states.data = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

    def _set_valve_state(self, index: int, value: int):
        if index < 0 or index > 15:
            raise Exception('Only index 0 to 15 allowed')
        if value < 0 or value > 1:
            raise Exception('Only values 0,1 are allowed')
        self.states.data[index] = value
    def set_data(self, valves_data: list):
        if len(valves_data) != 16:
            raise Exception('Data list should be exactly 16 elements')
        for v in valves_data:
            pass