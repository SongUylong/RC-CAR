import pygame
import sys
import serial
import time
import math
import random
import threading

# Import the 'inputs' library
try:
    from inputs import get_gamepad, UnpluggedError
except ImportError:
    print("The 'inputs' library is not installed.")
    print("Please run: pip install inputs")
    sys.exit(1)


# --- Configuration ---
SERIAL_PORT = "COM3"
BAUD_RATE = 115200
FPS = 60

CONTROLLER_CONFIG = {
    "STEERING_DEADZONE": 0.1,
    "THROTTLE_DEADZONE": 0.05,
    "STEERING_MULTIPLIER": 1.0,
    "TRIM_STEP": 0.02,
    # --- Steering Physics ---
    "STEERING_SMOOTHING_FACTOR": 0.8,
    "STEERING_EXPO": 3.0,
    "GEAR_1_REVERSE_LIMIT": 0.5,
}

KEEPALIVE_INTERVAL_MS = 250

# --- Visuals ---
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 600
COLOR_BACKGROUND = (30, 30, 40)
COLOR_ASPHALT_DARK = (45, 45, 55)
COLOR_ASPHALT_LIGHT = (60, 60, 70)
COLOR_WHITE = (240, 240, 240)
COLOR_GREY = (80, 80, 90)
COLOR_ACCENT = (0, 150, 255)
COLOR_SUCCESS = (0, 200, 100)
COLOR_FAIL = (255, 80, 80)
COLOR_WARN = (255, 180, 0)
COLOR_THROTTLE = (0, 220, 120)
COLOR_BRAKE = (220, 50, 50)
COLOR_REVERSE_LIGHT = (230, 230, 230)
COLOR_HEADLIGHT = (255, 255, 200)


# ==============================================================================
# CONTROLLER CLASS
# ==============================================================================
class Controller:
    def __init__(self):
        self.state = {
            "steer": 0.0,
            "accel": 0.0,
            "brake": 0.0,
            "gear": 0,
            "trim": 0.0,
            "buzzer": 0,  # +++ ADDED
        }
        self.connected = False
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()

    def _listen(self):
        print("🎮 Controller thread started. Waiting for gamepad...")
        while not self._stop_event.is_set():
            try:
                if not self.connected:
                    get_gamepad()
                    self.connected = True
                    print("✅ Controller connected.")

                events = get_gamepad()
                for event in events:
                    self._process_event(event)

            except UnpluggedError:
                print("⚠️ Controller unplugged.")
                self.connected = False
                # Reset all states except trim on disconnect
                for key in self.state:
                    if key != "trim":
                        self.state[key] = 0.0
                self.state["gear"] = 0
                self.state["buzzer"] = 0
                time.sleep(1)
            except Exception as e:
                print(f"An error occurred in the controller thread: {e}")
                self.connected = False
                time.sleep(2)

    def _process_event(self, event):
        # Mappings for a standard XInput controller (e.g., Xbox controller)
        if event.code == "ABS_X":  # Steering (Left Stick X-axis)
            self.state["steer"] = event.state / 32768.0
        elif event.code == "ABS_RZ":
            self.state["accel"] = event.state / 255.0
        elif event.code == "ABS_Z":
            self.state["brake"] = event.state / 255.0
        elif event.code == "BTN_TL":  # Gear 1 (Left Bumper)
            if event.state == 1:
                self.state["gear"] = 1
            elif self.state["gear"] == 1:
                self.state["gear"] = 0
        elif event.code == "BTN_TR":  # Gear 2 (Right Bumper)
            if event.state == 1:
                self.state["gear"] = 2
            elif self.state["gear"] == 2:
                self.state["gear"] = 0
        elif event.code == "BTN_NORTH" and event.state == 1:  # Trim Reset (Y button)
            self.state["trim"] = 0.0
        elif event.code == "ABS_HAT0X":  # Trim Adjust (D-Pad Left/Right)
            if event.state != 0:
                self.state["trim"] += event.state * CONTROLLER_CONFIG["TRIM_STEP"]
                self.state["trim"] = max(-1.0, min(1.0, self.state["trim"]))

        # +++ ADDED BUZZER LOGIC ('B' button on Xbox controller) +++
        elif event.code == "BTN_SOUTH":
            self.state["buzzer"] = event.state  # 1 for press, 0 for release


# ==============================================================================
# Car CLASS (No changes needed here)
# ==============================================================================
class Car:
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.body_width, self.body_height = 80, 150
        self.wheel_width, self.wheel_height = 18, 35
        self.steer_angle, self.wheel_rotation = 0, 0
        self.is_braking, self.is_reversing = False, False
        self.body_surface = self._create_body_surface()
        self.wheel_surface = self._create_wheel_surface()

    def _create_body_surface(self):
        surf = pygame.Surface((self.body_width, self.body_height), pygame.SRCALPHA)
        pygame.draw.rect(
            surf,
            COLOR_ACCENT,
            (0, 0, self.body_width, self.body_height),
            border_radius=15,
        )
        pygame.draw.rect(
            surf, (10, 10, 20), (10, 20, self.body_width - 20, 40), border_radius=8
        )
        pygame.draw.rect(surf, COLOR_HEADLIGHT, (5, 5, 20, 10), border_radius=5)
        pygame.draw.rect(
            surf, COLOR_HEADLIGHT, (self.body_width - 25, 5, 20, 10), border_radius=5
        )
        return surf

    def _create_wheel_surface(self):
        surf = pygame.Surface((self.wheel_width, self.wheel_height), pygame.SRCALPHA)
        pygame.draw.rect(
            surf,
            (20, 20, 20),
            (0, 0, self.wheel_width, self.wheel_height),
            border_radius=6,
        )
        pygame.draw.rect(
            surf,
            (40, 40, 40),
            (2, 2, self.wheel_width - 4, self.wheel_height - 4),
            2,
            6,
        )
        pygame.draw.circle(
            surf, COLOR_GREY, (self.wheel_width // 2, self.wheel_height // 2), 5
        )
        return surf

    def update(self, throttle, steer_output, gear, brake_val):
        self.is_reversing = brake_val > 0.1 and gear > 0
        self.is_braking = brake_val > 0.1 and gear > 0
        self.steer_angle = steer_output * 25
        rotation_speed = throttle * 10
        self.wheel_rotation = (self.wheel_rotation + rotation_speed) % 360

    def draw(self, screen):
        light_y_pos = self.y + self.body_height / 2 + 5
        if self.is_reversing:
            light_color = COLOR_REVERSE_LIGHT
            pygame.draw.rect(
                screen, light_color, (self.x - 30, light_y_pos, 20, 10), border_radius=5
            )
            pygame.draw.rect(
                screen, light_color, (self.x + 10, light_y_pos, 20, 10), border_radius=5
            )
        elif self.is_braking:
            light_color = COLOR_BRAKE
            pygame.draw.rect(
                screen, light_color, (self.x - 30, light_y_pos, 20, 10), border_radius=5
            )
            pygame.draw.rect(
                screen, light_color, (self.x + 10, light_y_pos, 20, 10), border_radius=5
            )

        screen.blit(
            self.body_surface,
            (self.x - self.body_width / 2, self.y - self.body_height / 2),
        )
        rotated_wheel = pygame.transform.rotate(self.wheel_surface, self.wheel_rotation)
        steered_wheel = pygame.transform.rotozoom(rotated_wheel, self.steer_angle, 1.0)
        wheel_positions = [
            (self.x - 45, self.y - 45),
            (self.x + 45, self.y - 45),
            (self.x - 45, self.y + 45),
            (self.x + 45, self.y + 45),
        ]
        screen.blit(steered_wheel, steered_wheel.get_rect(center=wheel_positions[0]))
        screen.blit(steered_wheel, steered_wheel.get_rect(center=wheel_positions[1]))
        screen.blit(rotated_wheel, rotated_wheel.get_rect(center=wheel_positions[2]))
        screen.blit(rotated_wheel, rotated_wheel.get_rect(center=wheel_positions[3]))


# ==============================================================================
# Dashboard CLASS
# ==============================================================================
class Dashboard:
    def __init__(self):
        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("RC Car Dashboard (Threaded Input)")
        self.clock = pygame.time.Clock()
        self.fonts = {
            "large": pygame.font.SysFont("Roboto", 48, bold=True),
            "medium": pygame.font.SysFont("Roboto", 24),
            "small": pygame.font.SysFont("Roboto", 18),
        }
        self.car = Car(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2 - 20)
        self.background_surface = self._create_asphalt_background()
        self.controller = Controller()
        self.ser = None
        self.status = {
            "joystick": "Connecting...",
            "serial": "Connecting...",
            "radio": "N/A",
        }
        self.last_sent_data = ""
        self.last_keepalive_time = 0
        self.current_steer_val = 0
        self.current_throttle_val = 0
        self.smoothed_steer = 0.0
        self._init_serial()

    def _init_serial(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            self.status["serial"] = f"Connected on {SERIAL_PORT}"
            time.sleep(2)
        except serial.SerialException:
            self.ser = None
            self.status["serial"] = f"ERROR: Port {SERIAL_PORT} not found"

    def _create_asphalt_background(self):
        bg = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
        bg.fill(COLOR_ASPHALT_DARK)
        for _ in range(300):
            pygame.draw.circle(
                bg,
                COLOR_ASPHALT_LIGHT,
                (random.randint(0, SCREEN_WIDTH), random.randint(0, SCREEN_HEIGHT)),
                random.randint(1, 2),
            )
        return bg

    def run(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            self.status["joystick"] = (
                "Connected" if self.controller.connected else "Not Connected"
            )

            # Unpack all values from processed input
            steer_output, throttle_output, brake_val, gear, buzzer_state = (
                self._get_processed_input()
            )

            steer_command = int(steer_output * 512)
            throttle_command = 0

            if gear > 0:
                if brake_val > CONTROLLER_CONFIG["THROTTLE_DEADZONE"]:
                    max_reverse_speed = (
                        CONTROLLER_CONFIG["GEAR_1_REVERSE_LIMIT"] if gear == 1 else 1.0
                    )
                    limited_brake_val = brake_val * max_reverse_speed
                    throttle_command = int(
                        -limited_brake_val * 512
                    )  # Note: Negative for reverse
                elif abs(throttle_output) > 0:
                    throttle_command = int(throttle_output * 512)

            self.current_steer_val = steer_command
            self.current_throttle_val = throttle_command

            self.car.update(throttle_output, steer_output, gear, brake_val)

            self._send_control_data(
                self.current_steer_val, self.current_throttle_val, buzzer_state
            )
            self._read_serial_feedback()
            self._draw_frame(steer_output, throttle_output, brake_val, gear)

            pygame.display.flip()
            self.clock.tick(FPS)

        self._cleanup()

    def _get_processed_input(self):
        raw_state = self.controller.state
        raw_steer = raw_state["steer"]
        target_steer = 0.0

        if abs(raw_steer) > CONTROLLER_CONFIG["STEERING_DEADZONE"]:
            expo = CONTROLLER_CONFIG["STEERING_EXPO"]
            steer_with_expo = math.copysign(pow(abs(raw_steer), expo), raw_steer)
            target_steer = steer_with_expo

        target_steer += raw_state["trim"]
        target_steer *= CONTROLLER_CONFIG["STEERING_MULTIPLIER"]
        target_steer = max(-1.0, min(1.0, target_steer))

        smoothing_factor = CONTROLLER_CONFIG["STEERING_SMOOTHING_FACTOR"]
        self.smoothed_steer += (target_steer - self.smoothed_steer) * smoothing_factor

        accel_val = raw_state["accel"]
        brake_val = raw_state["brake"]
        gear = raw_state["gear"]
        buzzer_state = raw_state["buzzer"]  # +++ ADDED

        throttle_output = 0.0
        if gear > 0 and accel_val > CONTROLLER_CONFIG["THROTTLE_DEADZONE"]:
            max_speed = 0.5 if gear == 1 else 1.0  # Note: Positive for forward
            throttle_output = accel_val * max_speed

        # Return all values, including the buzzer state
        return self.smoothed_steer, throttle_output, brake_val, gear, buzzer_state

    def _send_control_data(self, steer_val, throttle_val, buzzer_val):
        # +++ UPDATED data format to include buzzer state +++
        data_to_send = f"X:{steer_val},Y:{throttle_val},B:{buzzer_val}\n"
        current_time = pygame.time.get_ticks()
        data_has_changed = data_to_send != self.last_sent_data
        keepalive_due = current_time - self.last_keepalive_time > KEEPALIVE_INTERVAL_MS

        if (data_has_changed or keepalive_due) and self.ser and self.ser.is_open:
            try:
                self.ser.write(data_to_send.encode("utf-8"))
                self.last_sent_data = data_to_send
                self.last_keepalive_time = current_time
            except serial.SerialException:
                self.status["serial"] = "ERROR: Write Failed"
                self.ser.close()
                self.ser = None

    def _read_serial_feedback(self):
        if self.ser and self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if "TX FAILED" in line:
                    self.status["radio"] = "FAIL"
                elif line.startswith("RSSI:"):
                    parts = line.split(":")
                    if len(parts) > 1:
                        try:
                            rssi_val = int(parts[1])
                            if rssi_val > -70:
                                self.status["radio"] = "OK"
                            elif rssi_val > -85:
                                self.status["radio"] = "WEAK"
                            else:
                                self.status["radio"] = "POOR"
                        except ValueError:
                            self.status["radio"] = "Parse Error"
            except (serial.SerialException, TypeError) as e:
                self.status["radio"] = "Read Error"
                print(f"Serial read error: {e}")

    def _draw_frame(self, steer, throttle, brake, gear):
        self.screen.blit(self.background_surface, (0, 0))
        self.car.draw(self.screen)
        self._draw_text(
            "RC CAR DASHBOARD",
            self.fonts["large"],
            COLOR_WHITE,
            SCREEN_WIDTH / 2,
            40,
            center=True,
        )
        self._draw_gauge("Steering", steer, SCREEN_WIDTH / 4, 450, 100)

        bar_value = 0
        if gear > 0:
            # Throttle is positive, Brake (reverse) is negative
            bar_value = throttle if throttle != 0 else -brake
        self._draw_bar("Power", bar_value, (SCREEN_WIDTH / 4) * 3, 350, 50, 200)
        self._draw_status_box()

    def _draw_status_box(self):
        box_rect = pygame.Rect(10, SCREEN_HEIGHT - 120, SCREEN_WIDTH - 20, 110)
        pygame.draw.rect(self.screen, (20, 20, 30, 200), box_rect, 0, 10)

        joy_color = COLOR_SUCCESS if self.controller.connected else COLOR_FAIL
        self._draw_text(
            f"Controller: {self.status['joystick']}",
            self.fonts["medium"],
            joy_color,
            45,
            box_rect.y + 15,
        )
        pygame.draw.circle(self.screen, joy_color, (30, box_rect.y + 27), 8)

        serial_color = COLOR_SUCCESS if self.ser and self.ser.is_open else COLOR_FAIL
        self._draw_text(
            f"ESP32 Link: {self.status['serial']}",
            self.fonts["medium"],
            serial_color,
            45,
            box_rect.y + 45,
        )
        pygame.draw.circle(self.screen, serial_color, (30, box_rect.y + 57), 8)

        radio_status = self.status["radio"]
        radio_color = {
            "OK": COLOR_SUCCESS,
            "WEAK": COLOR_WARN,
            "POOR": COLOR_FAIL,
            "FAIL": COLOR_FAIL,
        }.get(radio_status, COLOR_GREY)
        self._draw_text(
            f"Radio Link: {radio_status}",
            self.fonts["medium"],
            radio_color,
            45,
            box_rect.y + 75,
        )
        pygame.draw.circle(self.screen, radio_color, (30, box_rect.y + 87), 8)

        # Display current buzzer state on dashboard
        buzzer_active = self.controller.state["buzzer"] == 1
        buzzer_color = COLOR_ACCENT if buzzer_active else COLOR_GREY
        self._draw_text(
            f"Buzzer: {'ON' if buzzer_active else 'OFF'}",
            self.fonts["medium"],
            buzzer_color,
            SCREEN_WIDTH - 400,
            box_rect.y + 15,
        )

        self._draw_text(
            f"Steer: {self.current_steer_val}",
            self.fonts["medium"],
            COLOR_WHITE,
            SCREEN_WIDTH - 200,
            box_rect.y + 15,
        )
        self._draw_text(
            f"Throttle: {self.current_throttle_val}",
            self.fonts["medium"],
            COLOR_WHITE,
            SCREEN_WIDTH - 200,
            box_rect.y + 45,
        )
        self._draw_text(
            f"Trim: {self.controller.state['trim']:.2f}",
            self.fonts["medium"],
            COLOR_WHITE,
            SCREEN_WIDTH - 200,
            box_rect.y + 75,
        )

        gear_map = {0: "N", 1: "1", 2: "2"}
        gear_text = gear_map.get(self.controller.state["gear"], "?")
        gear_color = COLOR_SUCCESS if self.controller.state["gear"] > 0 else COLOR_GREY
        gear_rect = pygame.Rect((SCREEN_WIDTH / 2) - 40, box_rect.y + 15, 80, 80)
        pygame.draw.rect(self.screen, (10, 10, 15), gear_rect, 0, 8)
        pygame.draw.rect(self.screen, gear_color, gear_rect, 2, 8)
        self._draw_text(
            "GEAR",
            self.fonts["small"],
            COLOR_GREY,
            gear_rect.centerx,
            gear_rect.y + 10,
            center=True,
        )
        self._draw_text(
            gear_text,
            self.fonts["large"],
            gear_color,
            gear_rect.centerx,
            gear_rect.centery + 5,
            center=True,
        )

    def _draw_text(self, text, font, color, x, y, center=False):
        surf = font.render(text, True, color)
        rect = surf.get_rect(center=(x, y)) if center else surf.get_rect(topleft=(x, y))
        self.screen.blit(surf, rect)

    def _draw_gauge(self, label, value, x, y, r):
        pygame.draw.arc(
            self.screen,
            COLOR_GREY,
            (x - r, y - r, r * 2, r * 2),
            math.pi * 0.75,
            math.pi * 2.25,
            5,
        )
        angle = (math.pi * 1.5) + (value * math.pi * 0.75)
        end_x, end_y = x + (r - 10) * math.cos(angle), y + (r - 10) * math.sin(angle)
        pygame.draw.line(self.screen, COLOR_ACCENT, (x, y), (end_x, end_y), 5)
        pygame.draw.circle(self.screen, COLOR_ACCENT, (x, y), 10)
        self._draw_text(
            label, self.fonts["medium"], COLOR_WHITE, x, y + r - 10, center=True
        )

    def _draw_bar(self, label, value, x, y, w, h):
        pygame.draw.rect(self.screen, COLOR_GREY, (x, y, w, h), 3, 5)
        fill_height = h * abs(value)
        bar_color = COLOR_THROTTLE if value > 0 else COLOR_BRAKE
        bar_rect = pygame.Rect(
            x, y + h - fill_height if value > 0 else y, w, fill_height
        )
        pygame.draw.rect(self.screen, bar_color, bar_rect)
        self._draw_text(
            label, self.fonts["medium"], COLOR_WHITE, x + w / 2, y + h + 20, center=True
        )

    def _cleanup(self):
        print("\n🔌 Shutting down...")
        self.controller.stop()
        if self.ser and self.ser.is_open:
            try:
                # Send a final stop command with buzzer off
                self.ser.write("X:0,Y:0,B:0\n".encode("utf-8"))
            except Exception as e:
                print(f"Could not send stop command: {e}")
            self.ser.close()
            print("Serial port closed.")
        pygame.quit()
        print("Pygame quit.")
        sys.exit()


if __name__ == "__main__":
    dashboard = None
    try:
        dashboard = Dashboard()
        dashboard.run()
    except KeyboardInterrupt:
        print("\n🛑 Script stopped by user.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        if dashboard:
            dashboard._cleanup()
