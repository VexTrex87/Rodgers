from vex import *
import time

brain = Brain()
controller = Controller()

front_left_wheel = Motor(Ports.PORT11, GearSetting.RATIO_6_1, False)
middle_left_wheel = Motor(Ports.PORT12, GearSetting.RATIO_6_1, False)
back_left_wheel = Motor(Ports.PORT13, GearSetting.RATIO_6_1, False)
left_wheels = MotorGroup(front_left_wheel, middle_left_wheel, back_left_wheel)

front_right_wheel = Motor(Ports.PORT14, GearSetting.RATIO_6_1, True)
middle_right_wheel = Motor(Ports.PORT15, GearSetting.RATIO_6_1, True)
back_right_wheel = Motor(Ports.PORT16, GearSetting.RATIO_6_1, True)
right_wheels = MotorGroup(front_right_wheel, middle_right_wheel, back_right_wheel)

inertial_sensor = Inertial(Ports.PORT19)
drivetrain = SmartDrive(left_wheels, right_wheels, inertial_sensor)
flywheel = Motor(Ports.PORT18, GearSetting.RATIO_6_1)
intake = Motor(Ports.PORT17, GearSetting.RATIO_18_1)
indexer = DigitalOut(brain.three_wire_port.a)
auton_selector = Bumper(brain.three_wire_port.b)
expansion = DigitalOut(brain.three_wire_port.c)

class Robot():
    def __init__(self):
        Competition(self.auton, self.auton)

        self.DRIVE_MULTIPLER = 1
        self.TURN_MULTIPLER = 0.7

        self.FLYWHEEL_FAR = 430
        self.FLYWHEEL_CLOSE = 335
        self.FLYWHEEL_OFF = 0
        self.flywheel_speed = 0
        self.FLYWHEEL_SPEED_DIFFERENCE = 30
        self.MAX_LAUNCHES = 45
        self.remaining_launches = int(self.MAX_LAUNCHES)

        self.selected_auton = 2
        self.autons = [
            {'name': 'LEFT SINGLE', 'action': self.left_single}, 
            {'name': 'LEFT DOUBLE', 'action': self.left_double}, 
            {'name': 'POG SKILLS', 'action': self.prog_skills},
            {'name': 'NO AUTON', 'action': self.no_auton},
        ]

        self.pre_auton()

    # auton

    def pre_auton(self):
        auton_selector.pressed(self.select_auton)

        drivetrain.set_stopping(COAST)

        ### inertial callibrate

        print('Ready')

        Thread(self.update_brain)
        Thread(self.flywheel_pid)

    def driver_controlled(self):
        controller.axis1.changed(self.on_controller_changed)
        controller.axis3.changed(self.on_controller_changed)

        controller.buttonL1.pressed(self.intake_on)
        controller.buttonL1.released(self.intake_off)
        controller.buttonR1.pressed(self.launch)
        controller.buttonUp.pressed(self.flywheel_close)
        controller.buttonDown.pressed(self.flywheel_off)
        controller.buttonX.pressed(self.expand)

        self.flywheel_close()

    def left_single(self):        
        # roller 1
        # launch discs

        pass

    def left_double(self):
        # roller
        # launch discs
        # roller 2

        pass

    def _get_position(self):
        wheel_positions = [
            front_left_wheel.position(DEGREES),
            middle_left_wheel.position(DEGREES),
            back_left_wheel.position(DEGREES),

            front_right_wheel.position(DEGREES),
            middle_right_wheel.position(DEGREES),
            back_right_wheel.position(DEGREES),
        ]

        average_position = sum(wheel_positions) / 6
        current_position = average_position / 32
        return current_position

    def move(self, duration, velocity):
        front_left_wheel.spin(FORWARD, velocity / 10.9, VOLT)
        middle_left_wheel.spin(FORWARD, velocity / 10.9, VOLT)
        back_left_wheel.spin(FORWARD, velocity / 10.9, VOLT)

        front_right_wheel.spin(FORWARD, velocity / 10.9, VOLT)
        middle_right_wheel.spin(FORWARD, velocity / 10.9, VOLT)
        back_right_wheel.spin(FORWARD, velocity / 10.9, VOLT)

        wait(duration)

        front_left_wheel.stop(COAST)
        middle_left_wheel.stop(COAST)
        back_left_wheel.stop(COAST)

        front_right_wheel.stop(COAST)
        middle_right_wheel.stop(COAST)
        back_right_wheel.stop(COAST)

    def turn(self, heading, velocity):
        front_left_wheel.spin(FORWARD, velocity / 10.9, VOLT)
        middle_left_wheel.spin(FORWARD, velocity / 10.9, VOLT)
        back_left_wheel.spin(FORWARD, velocity / 10.9, VOLT)

        front_right_wheel.spin(FORWARD, -velocity / 10.9, VOLT)
        middle_right_wheel.spin(FORWARD, -velocity / 10.9, VOLT)
        back_right_wheel.spin(FORWARD, -velocity / 10.9, VOLT)

        while True:
            current_heading = inertial_sensor.heading(DEGREES)
            if abs(current_heading - heading) < 2:
                front_left_wheel.stop(COAST)
                middle_left_wheel.stop(COAST)
                back_left_wheel.stop(COAST)

                front_right_wheel.stop(COAST)
                middle_right_wheel.stop(COAST)
                back_right_wheel.stop(COAST)

    def prog_skills(self):
        FAST_DRIVE_SPEED = 25
        DRIVE_SPEED = 10
        TURN_SPEED = 10
        INTAKE_DURATION = 0.4

        inertial_sensor.calibrate()
        while inertial_sensor.is_calibrating():
            wait(0.1, SECONDS)
        inertial_sensor.set_heading(0, DEGREES)

        # Move backward and get the back roller
        drivetrain.drive_for(FORWARD, 2, INCHES, DRIVE_SPEED, VOLT)
        intake.spin_for(FORWARD, INTAKE_DURATION, SECONDS, 100, PERCENT)

        # Move towards and get the left roller
        drivetrain.drive_for(REVERSE, 20, INCHES, DRIVE_SPEED, PERCENT)
        drivetrain.turn_to_heading(90, DEGREES, TURN_SPEED, PERCENT)

        drivetrain.drive_for(FORWARD, 23, INCHES, DRIVE_SPEED, PERCENT)
        intake.spin_for(FORWARD, INTAKE_DURATION, SECONDS, 100, PERCENT)

        return

        # Move towards the high goal while intaking, and launch discs
        drivetrain.drive_for(REVERSE, 9, INCHES, DRIVE_SPEED, PERCENT)
        drivetrain.turn_to_heading(225, DEGREES, TURN_SPEED, PERCENT)
        drivetrain.drive_for(FORWARD, 150, INCHES, FAST_DRIVE_SPEED, PERCENT)
        drivetrain.turn_to_heading(180, DEGREES, TURN_SPEED, PERCENT)

        # Move towards and get the top roller
        drivetrain.drive_for(FORWARD, 12, INCHES, DRIVE_SPEED, PERCENT)
        intake.spin_for(FORWARD, INTAKE_DURATION, SECONDS, 100, PERCENT)

        # Move towards and get the right roller
        drivetrain.drive_for(REVERSE, 24, INCHES, DRIVE_SPEED, PERCENT)
        drivetrain.turn_to_heading(270, DEGREES, TURN_SPEED, PERCENT)
        drivetrain.drive_for(FORWARD, 12, INCHES, DRIVE_SPEED, PERCENT)
        intake.spin_for(FORWARD, INTAKE_DURATION, SECONDS, 100, PERCENT)

        # Expand
        drivetrain.drive_for(REVERSE, 12, INCHES, DRIVE_SPEED, PERCENT)
        drivetrain.turn_to_heading(225, DEGREES, TURN_SPEED, PERCENT)
        wait(1, SECONDS)
        self.expand()

    def no_auton(self):
        pass

    def auton(self):
        start_time = time.time()
        self.autons[self.selected_auton]['action']()
        auton_duration = time.time() - start_time
        print('Auton took {} seconds'.format(auton_duration))

    def select_auton(self):
        if self.selected_auton == len(self.autons) - 1:
            self.selected_auton = 0
        else:
            self.selected_auton += 1

    # control

    def intake_on(self):
        intake.set_max_torque(100, PERCENT)
        intake.spin(FORWARD, 12, VOLT)

    def intake_off(self):
        intake.stop(COAST)

    def launch(self):
        self.remaining_launches -= 1
        indexer.set(True)
        wait(0.4, SECONDS)
        indexer.set(False)

    def expand(self):
        expansion.set(True)
        controller.rumble('-')

    def on_controller_changed(self):
        x_power = controller.axis1.position() * self.TURN_MULTIPLER
        y_power = controller.axis3.position() * self.DRIVE_MULTIPLER

        front_left_wheel.spin(FORWARD, (y_power + x_power) / 10.9, VOLT)
        middle_left_wheel.spin(FORWARD, (y_power + x_power) / 10.9, VOLT)
        back_left_wheel.spin(FORWARD, (y_power + x_power) / 10.9, VOLT)

        front_right_wheel.spin(FORWARD, (y_power - x_power) / 10.9, VOLT)
        middle_right_wheel.spin(FORWARD, (y_power - x_power) / 10.9, VOLT)
        back_right_wheel.spin(FORWARD, (y_power - x_power) / 10.9, VOLT)

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

        while True:
            velocity = round(flywheel.velocity(RPM))
            error = self.flywheel_speed - velocity
            total_error += error
            derivative = error - last_error
            power = (error * Kp) + (total_error * Ki) + (derivative * Kd)
            last_error = error

            if self.flywheel_speed == 0:
                flywheel.stop(COAST)
            else:
                flywheel.spin(FORWARD, power, VOLT)

            wait(0.1, SECONDS)

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
