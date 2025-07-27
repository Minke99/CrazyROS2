import pygame


class PygameJoystick:
    def __init__(self, ):
        pygame.init()
        pygame.joystick.init()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print('pygame started')
        self.name = self.joystick.get_name()
        print("Joystick name: {}".format(self.name))

        self.PS4_Controller = {'L1': 9, 'R1': 10,
                               'Triangle': 3, 'Circle': 1, 'Square': 2, 'Cross': 0,
                               'Option': 6, 'Share': 4, 'Touchpad': 15,
                               'LeftStick': 7, 'RightStick': 8,
                               'Up': 11, 'Down': 12, 'Left': 13, 'Right': 14,
                               'L2': 104, 'R2': 105,
                               'LeftStickX': 100, 'LeftStickY': 101, 'RightStickX': 102, 'RightStickY': 103, }

        self.PS4_Controller_MacUbuntu ={'Cross': 0, 'Circle': 1, 'Triangle': 2, 'Square': 3,
                                'Share': 4, 'Option': 6,
                                'L1': 9, 'R1': 10,
                                'LeftStickX': 100,    # axes[0]
                                'LeftStickY': 101,    # axes[1]
                                'L2': 102,   # axes[2]
                                'RightStickX': 103,            # axes[3]
                                'RightStickY': 104,            # axes[4]
                                'R2': 105,   # axes[5]
                                'Touchpad': 15, 'Up': 201, 'Down': 202, 'Left': 203, 'Right': 204,
                            }
        self.PS5_Controller = {'L1': 9, 'R1': 10,
                               'Triangle': 3, 'Circle': 1, 'Square': 2, 'Cross': 0,
                               'Option': 6, 'Share': 4, 'Touchpad': 15,
                               'LeftStick': 7, 'RightStick': 8,
                               'Up': 11, 'Down': 12, 'Left': 13, 'Right': 14,
                               'L2': 104, 'R2': 105,
                               'LeftStickX': 100, 'LeftStickY': 101, 'RightStickX': 102, 'RightStickY': 103, }
        self.X_Box_360 = {'L1': 4, 'R1': 5,
                          'Triangle': 3, 'Circle': 1, 'Square': 2, 'Cross': 0,
                          'Option': 7, 'Share': 6, 'Touchpad': 8,
                          'LeftStick': 9, 'RightStick': 10,
                          'Up': 201, 'Down': 202, 'Left': 203, 'Right': 204,
                          'L2': 102, 'R2': 105,
                          'LeftStickX': 100, 'LeftStickY': 101, 'RightStickX': 103, 'RightStickY': 104,
                          'TurnLeft': 4, 'TurnRight': 5}
        self.PS4_Controller_Ding = {'L1': 4, 'R1': 5,
                                    'Triangle': 2, 'Circle': 1, 'Square': 3, 'Cross': 0,
                                    'Option': 9, 'Share': 8, 'Touchpad': 8,
                                    'LeftStick': 11, 'RightStick': 12,
                                    'Up': 201, 'Down': 202, 'Left': 203, 'Right': 204,
                                    'L2': 102, 'R2': 105,
                                    'LeftStickX': 100, 'LeftStickY': 101, 'RightStickX': 103, 'RightStickY': 104, 
                                    'TurnLeft': 4, 'TurnRight': 5}

        self.KeyMapping = self.PS4_Controller

        if self.name == 'PS4 Controller':
            self.KeyMapping = self.PS4_Controller
            print('Use PS4 Controller Mapping')
        elif self.name == 'DualSense Wireless Controller':
            self.KeyMapping = self.PS4_Controller
            print('Use PS5 Controller Mapping in Macbook Air')
        elif self.name == 'Microsoft X-Box 360 pad':
            self.KeyMapping = self.X_Box_360
            print('Use X-Box 360 Controller Mapping')
        elif self.name == 'Xbox Wireless Controller':
            self.KeyMapping = self.X_Box_360
            print('Use X-Box 360 Controller Mapping')
        elif self.name == 'Wireless Controller':
            self.KeyMapping = self.PS4_Controller_Ding
            print('Use PS4 Controller Mapping (Ding)')
        elif self.name == 'Sony Interactive Entertainment Wireless Controller':
            self.KeyMapping = self.PS4_Controller_MacUbuntu
            print('Use PS4 Controller Mapping (MacUbuntu))')
        else:
            print('Warning: no suitable Key mapping')

    def step(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    def quit(self):
        pygame.quit()
        print('pygame stopped')

    def get_key(self, key):
        try:
            result = self.KeyMapping[key]
        except KeyError:
            print('Joystick key error')
            result = -1
        if 0 <= result < 100:
            return self.joystick.get_button(result)
        if 100 <= result < 200:
            return self.joystick.get_axis(result - 100)
        if 200 <= result < 300:
            hat = self.joystick.get_hat(0)
            if result == 201:  # up
                if hat[1] > 0:
                    return 1
                else:
                    return 0
            elif result == 202:  # down
                if -hat[1] > 0:
                    return 1
                else:
                    return 0
            elif result == 203:  # left
                if -hat[0] > 0:
                    return 1
                else:
                    return 0
            elif result == 204:  # right
                if hat[0] > 0:
                    return 1
                else:
                    return 0
        else:
            return 0

