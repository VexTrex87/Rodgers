from vex import *
import time

brain = Brain()
controller = Controller()

front_left_wheel = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
back_left_wheel = Motor(Ports.PORT13, GearSetting.RATIO_18_1, False)
left_wheels = MotorGroup(front_left_wheel, back_left_wheel)

front_right_wheel = Motor(Ports.PORT14, GearSetting.RATIO_18_1, True)
back_right_wheel = Motor(Ports.PORT16, GearSetting.RATIO_18_1, True)
right_wheels = MotorGroup(front_right_wheel, back_right_wheel)

inertial_sensor = Inertial(Ports.PORT19)
drivetrain = SmartDrive(left_wheels, right_wheels, inertial_sensor)
flywheel = Motor(Ports.PORT18, GearSetting.RATIO_6_1)
intake = Motor(Ports.PORT17, GearSetting.RATIO_6_1)
roller = Motor(Ports.PORT5, GearSetting.RATIO_18_1)
indexer = DigitalOut(brain.three_wire_port.a)
auton_selector = Bumper(brain.three_wire_port.b)
expansion = DigitalOut(brain.three_wire_port.c)

class Robot():
    def __init__(self):
        Competition(self.driver_controlled, self.auton)

        self.DRIVE_MULTIPLER = 1
        self.TURN_MULTIPLER = 1

        self.FLYWHEEL_FAR = 390
        self.FLYWHEEL_CLOSE = 360 # prev 335
        self.FLYWHEEL_OFF = 0
        self.flywheel_speed = 0
        self.is_pid_active = True
        self.IS_BUMP_ACTIVE = True
        self.FLYWHEEL_SPEED_DIFFERENCE = 30

        self.MAX_LAUNCHES = 45
        self.remaining_launches = int(self.MAX_LAUNCHES)

        self.selected_auton = 2
        self.autons = [
            {'name': 'LEFT SINGLE', 'action': self.left_single}, 
            {'name': 'LEFT DOUBLE', 'action': self.left_double}, 
            {'name': 'PROG SKILLS', 'action': self.prog_skills},
            {'name': 'NO AUTON', 'action': self.no_auton},
        ]

        self.pre_auton()

    # auton

    def pre_auton(self):
        auton_selector.pressed(self.select_auton)

        drivetrain.set_stopping(COAST)

        inertial_sensor.calibrate()
        while inertial_sensor.is_calibrating():
            wait(0.1, SECONDS)
        inertial_sensor.set_heading(0, DEGREES)

        print('Ready')

        Thread(self.update_brain)
        Thread(self.flywheel_pid)

    def driver_controlled(self):
        controller.axis1.changed(self.on_controller_changed)
        controller.axis3.changed(self.on_controller_changed)

        controller.buttonL1.pressed(self.intake_forward)
        controller.buttonL1.released(self.intake_off)
        controller.buttonL2.pressed(self.intake_reverse)
        controller.buttonL2.released(self.intake_off)
        controller.buttonR1.pressed(self.launch)
        controller.buttonR2.pressed(self.roller_on)
        controller.buttonR2.released(self.roller_off)
        controller.buttonRight.pressed(self.flywheel_far)
        controller.buttonUp.pressed(self.flywheel_close)
        controller.buttonDown.pressed(self.flywheel_off)
        controller.buttonX.pressed(self.expand)

        self.flywheel_close()

    def left_single(self):        
        # roller 1
        # launch discs

        pass

    def left_double(self):
        self.flywheel_close()

        #forward into roller?
        drivetrain.drive_for(FORWARD, 2, INCHES, 50, PERCENT)
        intake.spin_for(FORWARD, 0.4, SECONDS, 100, PERCENT)
        drivetrain.drive_for(REVERSE, 6, INCHES, 50, PERCENT)
        wait(1, SECONDS)

        drivetrain.turn_to_heading(-130, DEGREES, 20, PERCENT)
        wait(1, SECONDS)
    
        drivetrain.drive_for(FORWARD, 220, INCHES, 70, PERCENT)
        self.flywheel_far()
        drivetrain.turn_to_heading(320, DEGREES, 20, PERCENT)

        wait(1, SECONDS)
        self.launch()
        wait(2, SECONDS)
        self.launch()
        wait(2, SECONDS)
        self.launch()
        self.flywheel_off()

        drivetrain.turn_to_heading(-130, DEGREES, 20, PERCENT)
        drivetrain.drive_for(FORWARD, 220, INCHES, 50, PERCENT)
        drivetrain.turn_to_heading(270, DEGREES, 30, PERCENT)
        wait(1, SECONDS)

        drivetrain.drive_for(FORWARD, 10, INCHES, 50, PERCENT)
        intake.spin_for(FORWARD, 0.4, SECONDS, 100, PERCENT)

    def _move(self, distance, velocity):
        drivetrain.drive_for(FORWARD, distance, INCHES, velocity, PERCENT)

    def _turn(self, heading, velocity):
        drivetrain.turn_to_heading(heading, DEGREES, velocity, PERCENT)

    def _intake(self, duration):
        intake.spin_for(FORWARD, duration, SECONDS, 100, PERCENT)

    def _roller(self, rotation):
        roller.spin_for(FORWARD, rotation, DEGREES, 100, PERCENT)

    def prog_skills(self):
        FAST_DRIVE_SPEED = 60
        DRIVE_SPEED = 30
        TURN_SPEED = 30
        ROLLER_ROTATION = 140

        drivetrain.set_heading(0, DEGREES)
        drivetrain.set_timeout(2, SECONDS)
        roller.set_timeout(2, SECONDS)
        self.intake_forward()

        # Get roller 1
        self._move(2, DRIVE_SPEED)
        self._roller(ROLLER_ROTATION)

        # Get roller 2
        self._move(-21, DRIVE_SPEED)
        self._turn(90, TURN_SPEED)

        self.flywheel_speed = 450
        self._move(20, DRIVE_SPEED)
        self._roller(ROLLER_ROTATION)

        # Launch preloads
        self._move(-4, DRIVE_SPEED)
        self._turn(217, TURN_SPEED)
        self.intake_forward()
        self._move(14, DRIVE_SPEED)
        self._turn(0, TURN_SPEED)

        for volly in range(3):
            self.launch()
            wait(2, SECONDS)

        # Intake discs A
        self.flywheel_speed = 390
        self._turn(200, TURN_SPEED)
        self._move(20, DRIVE_SPEED)

        self._turn(225, TURN_SPEED)
        self._move(42, FAST_DRIVE_SPEED)

        # Launch discs A
        self._turn(215, TURN_SPEED)
        for volly in range(3):
            self.launch()
            wait(2, SECONDS)

        # Intake discs B
        self.flywheel_speed = 410
        self._turn(225, TURN_SPEED)
        self._move(28, FAST_DRIVE_SPEED)
        self._move(24, DRIVE_SPEED)

        # Launch discs B
        self._turn(260, TURN_SPEED)
        for volly in range(3):
            self.launch()
            wait(2, SECONDS)

        # Get roller 3
        self._turn(225, TURN_SPEED)
        self._move(30, FAST_DRIVE_SPEED)

        self._turn(180, TURN_SPEED)
        self._move(4, DRIVE_SPEED)
        
        self._roller(ROLLER_ROTATION)

        ## continue here ##

        # Get roller 4
        self._move(-18, DRIVE_SPEED)
        self._turn(270, TURN_SPEED)
        self._move(23, DRIVE_SPEED)
        self._roller(ROLLER_ROTATION)

        # Intake discs C
        pass

        # Launch discs C
        pass

        # Intake discs D
        pass

        # Launch discs D
        pass

        # Expand
        self._move(-12, DRIVE_SPEED)
        self._turn(225, TURN_SPEED)
        wait(1, SECONDS)
        self.expand()

    def no_auton(self):
        pass

    def auton(self):
        start_time = time.time()
        self.autons[self.selected_auton]['action']()
        auton_duration = time.time() - start_time
        print('Auton Elapsed Duration: {} sec'.format(auton_duration))

    def select_auton(self):
        if self.selected_auton == len(self.autons) - 1:
            self.selected_auton = 0
        else:
            self.selected_auton += 1

    # control

    def roller_on(self):
        roller.set_max_torque(100, PERCENT)
        roller.spin(FORWARD, 8.5, VOLT)

    def roller_off(self):
        roller.stop(COAST)

    def intake_forward(self):
        intake.set_max_torque(100, PERCENT)
        intake.spin(FORWARD, 12, VOLT)

    def intake_reverse(self):
        intake.set_max_torque(100, PERCENT)
        intake.spin(REVERSE, 12, VOLT)

    def intake_off(self):
        intake.stop(COAST)

    def launch(self):
        self.remaining_launches -= 1
        indexer.set(True)
        
        if self.IS_BUMP_ACTIVE == True:
            self.is_pid_active = False
            flywheel.spin(FORWARD, 12, VOLT)

        wait(0.15, SECONDS)
        indexer.set(False)

        # bump
        wait(0.15, SECONDS)
        self.is_pid_active = True

    def expand(self):
        expansion.set(True)
        controller.rumble('-')

    def on_controller_changed(self):
        x_power = controller.axis1.position() * self.TURN_MULTIPLER
        y_power = controller.axis3.position() * self.DRIVE_MULTIPLER

        front_left_wheel.spin(FORWARD, (y_power + x_power) / 12, VOLT)
        back_left_wheel.spin(FORWARD, (y_power + x_power) / 12, VOLT)
        front_right_wheel.spin(FORWARD, (y_power - x_power) / 12, VOLT)
        back_right_wheel.spin(FORWARD, (y_power - x_power) / 12, VOLT)

    # flywheel

    def flywheel_far(self):
        self.flywheel_speed = self.FLYWHEEL_FAR

    def flywheel_close(self):
        self.flywheel_speed = self.FLYWHEEL_CLOSE

    def flywheel_off(self):
        self.flywheel_speed = 0

    def flywheel_pid(self):
        Kp = 0.05
        Ki = 0.003
        Kd = 0

        last_error = 0
        total_error = 0

        velocities = []
        # while True:
        for second in range(30):
            wait(0.1, SECONDS)

            if self.is_pid_active == False:
                continue

            velocity = round(flywheel.velocity(RPM))
            velocities.append(velocity)

            error = self.flywheel_speed - velocity
            total_error += error
            derivative = error - last_error
            power = (error * Kp) + (total_error * Ki) + (derivative * Kd)
            last_error = error

            if self.flywheel_speed == 0:
                flywheel.stop(COAST)
            else:
                flywheel.spin(FORWARD, power, VOLT)

            if second in [20, 23, 26]:
                target = self.flywheel_speed
                actual = velocity
                error = round(((target - actual) / target) * 100)
                print('FIRE: {} / {} / {}'.format(target, actual, error))
                self.launch()

        print(*velocities)
        flywheel.stop(COAST)

    # brain

    def update_brain(self):
        while True:
            brain.screen.clear_screen()
            brain.screen.set_cursor(1, 1)
            brain.screen.set_font(FontType.MONO30)

            flywheel_velocity = flywheel.velocity(RPM)
            is_flywheel_ready = abs(flywheel_velocity - self.flywheel_speed) <= self.FLYWHEEL_SPEED_DIFFERENCE
            brain.screen.draw_rectangle(0, 0, 480, 240, is_flywheel_ready and Color.CYAN or Color.BLACK)

            values = [
                ['Drivetrain: ', drivetrain.temperature(PERCENT), 70, Color.YELLOW, Color.GREEN],
                ['Intake: ', intake.temperature(PERCENT), 70, Color.RED, Color.GREEN],
                ['Flywheel: ', flywheel.temperature(PERCENT), 70, Color.RED, Color.GREEN],
                ['Air', self.remaining_launches / self.MAX_LAUNCHES * 100, 15, Color.GREEN, Color.RED],
                ['Battery: ', brain.battery.capacity(), 20, Color.GREEN, Color.RED],
            ]

            for value in values:
                brain.screen.set_pen_color(value[1] >= value[2] and value[3] or value[4])
                brain.screen.print(value[0], round(value[1]))
                brain.screen.next_row()

            auton = self.autons[self.selected_auton]['name']
            brain.screen.set_pen_color(Color.WHITE)
            brain.screen.set_font(FontType.MONO60)
            brain.screen.print(auton)

            wait(0.25, SECONDS)

if __name__ == '__main__':
    robot = Robot()
