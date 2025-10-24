"""
3D Realistic Car Ground Movement Simulator with Bluetooth Control
Features: Detailed car model, realistic driving physics, prominent flat white card surface under the car, improved car design, and enhanced shadow for ground contact.

Save as: car_simulator_3d.py
Run with:
    pip install pygame numpy PyOpenGL PyOpenGL-accelerate pyserial
    python car_simulator_3d.py
"""

import sys
import math
import time
import threading
import re
import serial
import serial.tools.list_ports
import numpy as np
from collections import deque

try:
    import pygame
    from pygame.locals import *
    from OpenGL.GL import *
    from OpenGL.GLU import *
except Exception as e:
    print("Missing dependency:", e)
    print("Install with: pip install pygame numpy PyOpenGL PyOpenGL-accelerate pyserial")
    sys.exit(1)


# ---------------------- Bluetooth Controller ----------------------
class BluetoothController:
    """Bluetooth controller with smoothing and auto-reconnect."""

    def __init__(self, baudrate=38400, timeout=0.1, alpha=0.3):
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.connected = False
        self.running = False
        self.data_thread = None

        self.raw_data = {'x': 512, 'y': 512, 'button': 0}
        self.smoothed_data = {'x': 0.0, 'y': 0.0, 'button': 0}
        self.data_history = deque(maxlen=20)

        self.center_x = 512
        self.center_y = 512
        self.deadzone = 20
        self.max_deviation = 512.0

        self.roll_sensitivity = 1.0
        self.pitch_sensitivity = 1.0
        self.throttle_sensitivity = 0.5

        self._alpha = float(alpha)
        self._lock = threading.Lock()

        self._stop_reconnect = threading.Event()
        self._reconnect_thread = None

    def connect(self):
        """Scan COM ports and connect."""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        for p in ports:
            try:
                with serial.Serial(p, baudrate=self.baudrate, timeout=0.5) as ser:
                    for _ in range(4):
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        if not line:
                            continue
                        if re.match(r"^\s*X:\d+,\s*Y:\d+", line):
                            ser.close()
                            s = serial.Serial(p, baudrate=self.baudrate, timeout=self.timeout)
                            if s:
                                self.serial_conn = s
                                self.connected = True
                                print(f"✅ Connected to {p}")
                                return True
                            break
            except Exception:
                continue

        fallback = ["COM3", "COM4", "COM5", "/dev/ttyUSB0", "/dev/ttyACM0"]
        for p in fallback:
            try:
                s = serial.Serial(p, baudrate=self.baudrate, timeout=self.timeout)
                self.serial_conn = s
                self.connected = True
                print(f"✅ Connected to {p}")
                return True
            except Exception:
                continue

        self.connected = False
        return False

    def start_listening(self):
        if not self.connected or not self.serial_conn:
            return False
        self.running = True
        self.data_thread = threading.Thread(target=self._data_loop, daemon=True)
        self.data_thread.start()
        return True

    def stop(self):
        self.running = False
        self._stop_reconnect.set()
        if self.data_thread and self.data_thread.is_alive():
            self.data_thread.join(timeout=0.5)
        try:
            if self.serial_conn and getattr(self.serial_conn, 'is_open', False):
                self.serial_conn.close()
        except Exception:
            pass
        self.connected = False

    def _data_loop(self):
        try:
            while self.running and self.serial_conn:
                try:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                except Exception:
                    line = ''
                if not line:
                    time.sleep(0.005)
                    continue

                try:
                    if ':' in line:
                        parts = [p.strip() for p in line.split(',') if p.strip()]
                        found_x = found_y = None
                        for part in parts:
                            if ':' in part:
                                k, v = part.split(':', 1)
                                k = k.strip().lower()
                                v = v.strip()
                                if k == 'x':
                                    found_x = int(float(v))
                                elif k == 'y':
                                    found_y = int(float(v))
                        if found_x is not None and found_y is not None:
                            self._process_raw_data(found_x, found_y)
                    else:
                        parts = [p.strip() for p in line.split(',') if p.strip()]
                        if len(parts) >= 2:
                            x = int(float(parts[0]))
                            y = int(float(parts[1]))
                            self._process_raw_data(x, y)
                except Exception:
                    continue
        except Exception:
            self.connected = False

    def _process_raw_data(self, x, y):
        with self._lock:
            self.raw_data['x'] = int(x)
            self.raw_data['y'] = int(y)

            x_diff = self.raw_data['x'] - self.center_x
            y_diff = self.raw_data['y'] - self.center_y

            if abs(x_diff) < self.deadzone:
                x_diff = 0
            if abs(y_diff) < self.deadzone:
                y_diff = 0

            x_norm = float(np.clip(x_diff / self.max_deviation, -1.0, 1.0))
            y_norm = float(np.clip(-y_diff / self.max_deviation, -1.0, 1.0))

            prev_x = self.smoothed_data.get('x', 0.0)
            prev_y = self.smoothed_data.get('y', 0.0)
            self.smoothed_data['x'] = self._alpha * x_norm + (1 - self._alpha) * prev_x
            self.smoothed_data['y'] = self._alpha * y_norm + (1 - self._alpha) * prev_y

    def get_smoothed_input(self):
        with self._lock:
            sx = int(self.smoothed_data['x'] * 512 + 512)
            sy = int(self.smoothed_data['y'] * 512 + 512)
            return {'x': sx, 'y': sy, 'button': int(self.raw_data.get('button', 0))}

    def get_control_inputs(self):
        with self._lock:
            steering = float(self.smoothed_data.get('x', 0.0))
            throttle = float(self.smoothed_data.get('y', 0.0))
            return {'steering': steering, 'throttle': throttle}

    def start_auto_reconnect(self, check_interval=2.0):
        if self._reconnect_thread and self._reconnect_thread.is_alive():
            return
        self._stop_reconnect.clear()
        self._reconnect_thread = threading.Thread(target=self._auto_reconnect_loop, args=(check_interval,), daemon=True)
        self._reconnect_thread.start()

    def _auto_reconnect_loop(self, check_interval):
        while not self._stop_reconnect.is_set():
            if self.connected and self.serial_conn and getattr(self.serial_conn, 'is_open', True):
                time.sleep(check_interval)
                continue
            try:
                print("Attempting to reconnect...")
                ok = self.connect()
                if ok:
                    if not (self.data_thread and self.data_thread.is_alive()):
                        self.start_listening()
                else:
                    time.sleep(check_interval)
            except Exception:
                time.sleep(check_interval)


# ---------------------- Car Simulator ----------------------
class CarSimulator3D:
    def __init__(self):
        pygame.init()
        self.display = (1200, 800)

        pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLEBUFFERS, 1)
        pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLESAMPLES, 4)
        pygame.display.gl_set_attribute(pygame.GL_DEPTH_SIZE, 24)
        pygame.display.gl_set_attribute(pygame.GL_DOUBLEBUFFER, 1)

        self.screen = pygame.display.set_mode(self.display, DOUBLEBUF | OPENGL)
        pygame.display.set_caption("3D Car Simulator - Bluetooth Control")

        self.init_opengl()

        # Car state
        self.position = np.array([0.0, 0.4, 0.0], dtype=np.float32)  # Start at center of road network
        self.velocity = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.yaw = 0.0
        self.ground_level = 0.4  # Ensure car sits close to ground

        # Car physics
        self.speed = 0.0
        self.steering_angle = 0.0
        self.wheel_rotation = 0.0

        # Controls
        self.throttle_input = 0.0
        self.steering_input = 0.0

        # Camera
        self.camera_modes = ['chase', 'side', 'top', 'hood']
        self.camera_index = 0

        # Terrain
        self.terrain_size = 1000  # Much larger terrain
        self.clouds = []
        self.generate_clouds()

        self.clock = pygame.time.Clock()
        self.running = True
        self.fps = 60

        # Bluetooth
        self.bt_controller = BluetoothController()
        self.bt_connected = False
        self._setup_bluetooth()

        # Display lists
        self.car_list = None
        self.terrain_list = None
        self.cloud_list = None
        self.display_lists_ready = False

        self.font = pygame.font.SysFont('Consolas', 16)

        print("✅ 3D Car Simulator Initialized")
        print("🚗 Controls: Arrow Keys (steer/gas), C (camera), R (reset)")

    def init_opengl(self):
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_LIGHT1)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glShadeModel(GL_SMOOTH)
        glEnable(GL_MULTISAMPLE)
        glEnable(GL_CULL_FACE)
        glCullFace(GL_BACK)
        glClearColor(0.2, 0.3, 0.5, 1.0)  # Dark blue-gray background for better contrast

        glEnable(GL_FOG)
        glFogfv(GL_FOG_COLOR, (0.2, 0.3, 0.5, 1.0))  # Match background color
        glFogi(GL_FOG_MODE, GL_LINEAR)
        glFogf(GL_FOG_START, 200.0)
        glFogf(GL_FOG_END, 800.0)

        glLightfv(GL_LIGHT0, GL_POSITION, (100.0, 200.0, 100.0, 1.0))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.4, 0.4, 0.4, 1.0))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (1.0, 0.95, 0.8, 1.0))
        glLightfv(GL_LIGHT0, GL_SPECULAR, (1.0, 1.0, 1.0, 1.0))

        glLightfv(GL_LIGHT1, GL_POSITION, (-50.0, 100.0, -50.0, 1.0))
        glLightfv(GL_LIGHT1, GL_AMBIENT, (0.2, 0.2, 0.3, 1.0))
        glLightfv(GL_LIGHT1, GL_DIFFUSE, (0.6, 0.7, 0.9, 1.0))

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(65.0, (self.display[0] / float(self.display[1])), 0.1, 5000.0)
        glMatrixMode(GL_MODELVIEW)

    def _setup_bluetooth(self):
        try:
            ok = self.bt_controller.connect()
            if ok:
                self.bt_controller.start_listening()
                self.bt_controller.start_auto_reconnect()
                self.bt_connected = True
            else:
                print("⚠️  Bluetooth not found - auto-reconnect started")
                self.bt_controller.start_auto_reconnect()
                self.bt_connected = False
        except Exception as e:
            print("❌ Bluetooth Error:", e)
            self.bt_controller.start_auto_reconnect()
            self.bt_connected = False

    def generate_clouds(self):
        np.random.seed(1)
        for _ in range(30):
            x = np.random.uniform(-800, 800)
            y = np.random.uniform(60, 180)
            z = np.random.uniform(-800, 800)
            s = np.random.uniform(5, 15)
            self.clouds.append((x, y, z, s))

    def process_bluetooth_input(self):
        self.bt_connected = self.bt_controller.connected
        if not self.bt_connected:
            return
        try:
            ci = self.bt_controller.get_control_inputs()
            self.steering_input = ci['steering']
            self.throttle_input = ci['throttle']
        except Exception:
            pass

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == QUIT:
                self.running = False
            elif event.type == KEYDOWN:
                if event.key == K_c:
                    self.camera_index = (self.camera_index + 1) % len(self.camera_modes)
                elif event.key == K_r:
                    self.reset_car()
                elif event.key == K_ESCAPE:
                    self.running = False

        keys = pygame.key.get_pressed()
        
        # Keyboard steering
        if keys[K_LEFT]:
            self.steering_input = -1.0
        elif keys[K_RIGHT]:
            self.steering_input = 1.0
        else:
            if not self.bt_connected:
                self.steering_input *= 0.8  # Smooth return to center

        # Keyboard throttle
        if keys[K_UP]:
            self.throttle_input = 1.0
        elif keys[K_DOWN]:
            self.throttle_input = -0.5  # Reverse
        else:
            if not self.bt_connected:
                self.throttle_input *= 0.9  # Smooth deceleration

    def reset_car(self):
        self.position = np.array([0.0, 0.4, 0.0], dtype=np.float32)  # Start at center of road network
        self.velocity = np.zeros(3, dtype=np.float32)
        self.yaw = 0.0
        self.speed = 0.0
        self.steering_angle = 0.0
        self.wheel_rotation = 0.0

    def update_physics(self):
        self.process_bluetooth_input()

        # Car physics constants
        acceleration = 0.08
        max_speed = 3.0
        friction = 0.98
        turn_rate = 2.5

        # Apply throttle
        self.speed += self.throttle_input * acceleration
        self.speed *= friction

        # Limit speed
        self.speed = np.clip(self.speed, -max_speed * 0.5, max_speed)

        # Steering (only when moving)
        if abs(self.speed) > 0.1:
            self.steering_angle = self.steering_input * 35.0
            self.yaw += self.steering_angle * (self.speed / max_speed) * 0.1
        else:
            self.steering_angle *= 0.8

        # Calculate movement direction
        yaw_rad = math.radians(self.yaw)
        forward = np.array([
            math.sin(yaw_rad),
            0.0,
            math.cos(yaw_rad)
        ], dtype=np.float32)

        # Update velocity
        self.velocity = forward * self.speed

        # Update position
        self.position += self.velocity

        # Keep on ground
        self.position[1] = self.ground_level

        # Update wheel rotation based on speed
        self.wheel_rotation += self.speed * 10.0
        self.wheel_rotation %= 360

        # Boundary check
        terrain_limit = self.terrain_size * 0.9  # Allow driving almost to the edge
        if abs(self.position[0]) > terrain_limit:
            self.position[0] = terrain_limit if self.position[0] > 0 else -terrain_limit
            self.velocity[0] = 0
            self.speed = 0
        if abs(self.position[2]) > terrain_limit:
            self.position[2] = terrain_limit if self.position[2] > 0 else -terrain_limit
            self.velocity[2] = 0
            self.speed = 0

    def build_display_lists(self):
        if self.display_lists_ready:
            return

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        # Terrain - Large road network
        self.terrain_list = glGenLists(1)
        glNewList(self.terrain_list, GL_COMPILE)
        glEnable(GL_LIGHTING)  # Enable lighting for terrain

        s = self.terrain_size

        # Massive grass field base
        glBegin(GL_QUADS)
        glColor3f(0.2, 0.7, 0.2)  # Brighter green grass color
        glNormal3f(0, 1, 0)
        glVertex3f(-s, 0, -s)
        glVertex3f(s, 0, -s)
        glVertex3f(s, 0, s)
        glVertex3f(-s, 0, s)
        glEnd()
        
        # Add grass texture pattern across the entire field
        glColor3f(0.3, 0.5, 0.2)  # Darker green for texture
        step_size = s / 50  # More detailed texture
        for i in range(-25, 26):
            for j in range(-25, 26):
                if (i + j) % 2 == 0:  # Checkerboard pattern
                    x1, z1 = i * step_size, j * step_size
                    x2, z2 = (i + 1) * step_size, (j + 1) * step_size
                    glBegin(GL_QUADS)
                    glVertex3f(x1, 0.001, z1)
                    glVertex3f(x2, 0.001, z1)
                    glVertex3f(x2, 0.001, z2)
                    glVertex3f(x1, 0.001, z2)
                    glEnd()

        # Main North-South Highway (wide)
        road_width = 20
        step = s / 100
        glBegin(GL_QUADS)
        for i in range(-50, 51):
            z1 = i * step
            z2 = (i + 1) * step
            glColor3f(0.1, 0.1, 0.15)  # Very dark asphalt color for contrast
            glNormal3f(0, 1, 0)
            glVertex3f(-road_width, 0.01, z1)
            glVertex3f(road_width, 0.01, z1)
            glVertex3f(road_width, 0.01, z2)
            glVertex3f(-road_width, 0.01, z2)
        glEnd()
        
        # Main East-West Highway (wide)
        glBegin(GL_QUADS)
        for i in range(-50, 51):
            x1 = i * step
            x2 = (i + 1) * step
            glColor3f(0.1, 0.1, 0.15)  # Very dark asphalt color for contrast
            glNormal3f(0, 1, 0)
            glVertex3f(x1, 0.01, -road_width)
            glVertex3f(x2, 0.01, -road_width)
            glVertex3f(x2, 0.01, road_width)
            glVertex3f(x1, 0.01, road_width)
        glEnd()
        
        # Secondary roads (narrower)
        secondary_width = 12
        # North-South secondary roads
        for offset in [-60, -30, 30, 60]:
            glBegin(GL_QUADS)
            for i in range(-50, 51):
                z1 = i * step
                z2 = (i + 1) * step
                glColor3f(0.25, 0.25, 0.3)  # Slightly lighter asphalt
                glNormal3f(0, 1, 0)
                glVertex3f(offset - secondary_width/2, 0.01, z1)
                glVertex3f(offset + secondary_width/2, 0.01, z1)
                glVertex3f(offset + secondary_width/2, 0.01, z2)
                glVertex3f(offset - secondary_width/2, 0.01, z2)
            glEnd()
        
        # East-West secondary roads
        for offset in [-60, -30, 30, 60]:
            glBegin(GL_QUADS)
            for i in range(-50, 51):
                x1 = i * step
                x2 = (i + 1) * step
                glColor3f(0.25, 0.25, 0.3)  # Slightly lighter asphalt
                glNormal3f(0, 1, 0)
                glVertex3f(x1, 0.01, offset - secondary_width/2)
                glVertex3f(x2, 0.01, offset - secondary_width/2)
                glVertex3f(x2, 0.01, offset + secondary_width/2)
                glVertex3f(x1, 0.01, offset + secondary_width/2)
            glEnd()

        # Road markings - Main highways
        glColor3f(1.0, 1.0, 0.0)  # Bright yellow lines
        glBegin(GL_QUADS)
        for i in range(-50, 51):
            z1 = i * step
            z2 = (i + 1) * step
            if i % 3 == 0:  # Dashed center line
                glVertex3f(-1.0, 0.02, z1)
                glVertex3f(1.0, 0.02, z1)
                glVertex3f(1.0, 0.02, z1 + step * 0.7)
                glVertex3f(-1.0, 0.02, z1 + step * 0.7)
        glEnd()
        
        # White road edges for main highways
        glColor3f(0.9, 0.9, 0.9)  # White road edges
        glBegin(GL_QUADS)
        for i in range(-50, 51):
            z1 = i * step
            z2 = (i + 1) * step
            # Left edge
            glVertex3f(-road_width, 0.02, z1)
            glVertex3f(-road_width + 1.0, 0.02, z1)
            glVertex3f(-road_width + 1.0, 0.02, z2)
            glVertex3f(-road_width, 0.02, z2)
            # Right edge
            glVertex3f(road_width - 1.0, 0.02, z1)
            glVertex3f(road_width, 0.02, z1)
            glVertex3f(road_width, 0.02, z2)
            glVertex3f(road_width - 1.0, 0.02, z2)
        glEnd()

        glEndList()
        print("✅ Terrain list built successfully")

        # Clouds
        self.cloud_list = glGenLists(1)
        glNewList(self.cloud_list, GL_COMPILE)
        glDisable(GL_LIGHTING)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        for c in self.clouds:
            x, y, z, sz = c
            glPushMatrix()
            glTranslatef(x, y, z)
            glColor4f(1.0, 1.0, 1.0, 0.7)
            quad = gluNewQuadric()
            gluSphere(quad, sz, 10, 6)
            glPopMatrix()

        glDisable(GL_BLEND)
        glEnable(GL_LIGHTING)
        glEndList()

        self.display_lists_ready = True

    def draw_car(self):
        """Draw improved realistic car model with more details"""
        quad = gluNewQuadric()

        # Car body - main chassis (sleeker design)
        glPushMatrix()
        glColor3f(0.1, 0.1, 0.8)  # Blue car body
        glTranslatef(0, 0.4, 0)
        glScalef(2.0, 0.5, 4.0)
        self._draw_rounded_box()
        glPopMatrix()

        # Cabin/roof (tapered)
        glPushMatrix()
        glColor3f(0.05, 0.05, 0.7)
        glTranslatef(0, 0.9, -0.5)
        glScalef(1.6, 0.6, 2.0)
        self._draw_rounded_box()
        glPopMatrix()

        # Windows - front windshield (slanted)
        glPushMatrix()
        glColor3f(0.2, 0.2, 0.3)
        glTranslatef(0, 1.0, 0.8)
        glRotatef(30, 1, 0, 0)
        glScalef(1.5, 0.5, 0.1)
        self._draw_rounded_box()
        glPopMatrix()

        # Windows - back
        glPushMatrix()
        glColor3f(0.2, 0.2, 0.3)
        glTranslatef(0, 1.0, -1.5)
        glRotatef(-20, 1, 0, 0)
        glScalef(1.5, 0.5, 0.1)
        self._draw_rounded_box()
        glPopMatrix()

        # Side windows left
        glPushMatrix()
        glColor3f(0.2, 0.2, 0.3)
        glTranslatef(-0.81, 0.9, -0.3)
        glScalef(0.1, 0.6, 2.0)
        self._draw_rounded_box()
        glPopMatrix()

        # Side windows right
        glPushMatrix()
        glColor3f(0.2, 0.2, 0.3)
        glTranslatef(0.81, 0.9, -0.3)
        glScalef(0.1, 0.6, 2.0)
        self._draw_rounded_box()
        glPopMatrix()

        # Hood (with vents)
        glPushMatrix()
        glColor3f(0.1, 0.1, 0.8)
        glTranslatef(0, 0.6, 1.6)
        glScalef(1.9, 0.2, 1.0)
        self._draw_rounded_box()
        # Vents
        glColor3f(0.1, 0.1, 0.1)
        glTranslatef(0.5, 0.1, 0.3)
        glScalef(0.2, 0.5, 0.3)
        self._draw_rounded_box()
        glTranslatef(-5, 0, 0)
        self._draw_rounded_box()
        glPopMatrix()

        # Trunk
        glPushMatrix()
        glColor3f(0.1, 0.1, 0.8)
        glTranslatef(0, 0.6, -1.8)
        glScalef(1.9, 0.2, 0.8)
        self._draw_rounded_box()
        glPopMatrix()

        # Front bumper with grill
        glPushMatrix()
        glColor3f(0.2, 0.2, 0.2)
        glTranslatef(0, 0.25, 2.2)
        glScalef(2.1, 0.3, 0.4)
        self._draw_rounded_box()
        # Grill
        glColor3f(0.1, 0.1, 0.1)
        glTranslatef(0, 0, 0.1)
        glScalef(0.8, 0.8, 0.1)
        self._draw_rounded_box()
        glPopMatrix()

        # Rear bumper
        glPushMatrix()
        glColor3f(0.2, 0.2, 0.2)
        glTranslatef(0, 0.25, -2.3)
        glScalef(2.1, 0.3, 0.4)
        self._draw_rounded_box()
        glPopMatrix()

        # Headlights (LED style)
        glPushMatrix()
        glColor3f(1.0, 1.0, 0.9)
        glTranslatef(0.7, 0.4, 2.2)
        glScalef(0.4, 0.2, 0.1)
        self._draw_rounded_box()
        glPopMatrix()
        glPushMatrix()
        glTranslatef(-0.7, 0.4, 2.2)
        glScalef(0.4, 0.2, 0.1)
        self._draw_rounded_box()
        glPopMatrix()

        # Tail lights (modern strip)
        glPushMatrix()
        glColor3f(0.9, 0.1, 0.1)
        glTranslatef(0, 0.4, -2.3)
        glScalef(1.8, 0.15, 0.1)
        self._draw_rounded_box()
        glPopMatrix()

        # Spoiler
        glPushMatrix()
        glColor3f(0.1, 0.1, 0.8)
        glTranslatef(0, 1.3, -2.2)
        glScalef(1.8, 0.1, 0.4)
        self._draw_rounded_box()
        glPopMatrix()
        # Spoiler supports
        glPushMatrix()
        glColor3f(0.2, 0.2, 0.2)
        glTranslatef(0.7, 0.9, -2.0)
        glRotatef(90, 1, 0, 0)
        gluCylinder(quad, 0.05, 0.05, 0.4, 6, 2)
        glPopMatrix()
        glPushMatrix()
        glTranslatef(-0.7, 0.9, -2.0)
        glRotatef(90, 1, 0, 0)
        gluCylinder(quad, 0.05, 0.05, 0.4, 6, 2)
        glPopMatrix()

        # Exhaust pipes
        glPushMatrix()
        glColor3f(0.3, 0.3, 0.3)
        glTranslatef(0.6, 0.3, -2.3)
        glRotatef(90, 1, 0, 0)
        gluCylinder(quad, 0.12, 0.12, 0.4, 8, 2)
        glPopMatrix()
        glPushMatrix()
        glTranslatef(-0.6, 0.3, -2.3)
        glRotatef(90, 1, 0, 0)
        gluCylinder(quad, 0.12, 0.12, 0.4, 8, 2)
        glPopMatrix()

        # Side skirts
        glPushMatrix()
        glColor3f(0.2, 0.2, 0.2)
        glTranslatef(1.0, 0.2, 0)
        glScalef(0.1, 0.2, 3.5)
        self._draw_rounded_box()
        glPopMatrix()
        glPushMatrix()
        glTranslatef(-1.0, 0.2, 0)
        glScalef(0.1, 0.2, 3.5)
        self._draw_rounded_box()
        glPopMatrix()

        # Wheels - Front Left
        glPushMatrix()
        glTranslatef(-1.1, 0.35, 1.4)
        glRotatef(self.steering_angle, 0, 1, 0)
        glRotatef(self.wheel_rotation, 1, 0, 0)
        self._draw_wheel()
        glPopMatrix()

        # Front Right
        glPushMatrix()
        glTranslatef(1.1, 0.35, 1.4)
        glRotatef(self.steering_angle, 0, 1, 0)
        glRotatef(self.wheel_rotation, 1, 0, 0)
        self._draw_wheel()
        glPopMatrix()

        # Rear Left
        glPushMatrix()
        glTranslatef(-1.1, 0.35, -1.4)
        glRotatef(self.wheel_rotation, 1, 0, 0)
        self._draw_wheel()
        glPopMatrix()

        # Rear Right
        glPushMatrix()
        glTranslatef(1.1, 0.35, -1.4)
        glRotatef(self.wheel_rotation, 1, 0, 0)
        self._draw_wheel()
        glPopMatrix()

        # Side mirrors
        glPushMatrix()
        glColor3f(0.2, 0.2, 0.2)
        glTranslatef(-1.6, 0.9, 0.5)
        glScalef(0.2, 0.15, 0.4)
        self._draw_rounded_box()
        glPopMatrix()
        glPushMatrix()
        glTranslatef(1.6, 0.9, 0.5)
        glScalef(0.2, 0.15, 0.4)
        self._draw_rounded_box()
        glPopMatrix()

    def _draw_wheel(self):
        """Draw an improved car wheel with tire, rim, spokes, and hub"""
        quad = gluNewQuadric()
        
        # Tire (black rubber)
        glColor3f(0.1, 0.1, 0.1)
        glPushMatrix()
        glRotatef(90, 0, 1, 0)
        gluCylinder(quad, 0.4, 0.4, 0.3, 12, 2)
        gluDisk(quad, 0, 0.4, 12, 2)
        glTranslatef(0, 0, 0.3)
        gluDisk(quad, 0, 0.4, 12, 2)
        glPopMatrix()
        
        # Rim (metallic)
        glColor3f(0.6, 0.6, 0.7)
        glPushMatrix()
        glTranslatef(0, 0, 0.15)
        glRotatef(90, 0, 1, 0)
        gluDisk(quad, 0, 0.3, 8, 2)
        glPopMatrix()

        # Spokes
        glColor3f(0.5, 0.5, 0.5)
        glPushMatrix()
        glTranslatef(0, 0, 0.15)
        glBegin(GL_LINES)
        for i in range(8):
            angle = i * 45
            x = math.cos(math.radians(angle)) * 0.3
            y = math.sin(math.radians(angle)) * 0.3
            glVertex3f(0, 0, 0)
            glVertex3f(x, y, 0)
        glEnd()
        glPopMatrix()

        # Central hub
        glColor3f(0.4, 0.4, 0.4)
        glPushMatrix()
        glTranslatef(0, 0, 0.15)
        glRotatef(90, 0, 1, 0)
        gluDisk(quad, 0, 0.12, 8, 2)
        glPopMatrix()

    def _draw_rounded_box(self):
        """Draw a unit box (placeholder for rounded, but kept as is for performance)"""
        glBegin(GL_QUADS)
        # Front
        glNormal3f(0,0,1)
        glVertex3f(-0.5,-0.5,0.5)
        glVertex3f(0.5,-0.5,0.5)
        glVertex3f(0.5,0.5,0.5)
        glVertex3f(-0.5,0.5,0.5)
        # Back
        glNormal3f(0,0,-1)
        glVertex3f(-0.5,-0.5,-0.5)
        glVertex3f(-0.5,0.5,-0.5)
        glVertex3f(0.5,0.5,-0.5)
        glVertex3f(0.5,-0.5,-0.5)
        # Top
        glNormal3f(0,1,0)
        glVertex3f(-0.5,0.5,-0.5)
        glVertex3f(-0.5,0.5,0.5)
        glVertex3f(0.5,0.5,0.5)
        glVertex3f(0.5,0.5,-0.5)
        # Bottom
        glNormal3f(0,-1,0)
        glVertex3f(-0.5,-0.5,-0.5)
        glVertex3f(0.5,-0.5,-0.5)
        glVertex3f(0.5,-0.5,0.5)
        glVertex3f(-0.5,-0.5,0.5)
        # Left
        glNormal3f(-1,0,0)
        glVertex3f(-0.5,-0.5,-0.5)
        glVertex3f(-0.5,-0.5,0.5)
        glVertex3f(-0.5,0.5,0.5)
        glVertex3f(-0.5,0.5,-0.5)
        # Right
        glNormal3f(1,0,0)
        glVertex3f(0.5,-0.5,-0.5)
        glVertex3f(0.5,0.5,-0.5)
        glVertex3f(0.5,0.5,0.5)
        glVertex3f(0.5,-0.5,0.5)
        glEnd()

    def update_camera(self):
        mode = self.camera_modes[self.camera_index]
        pos = self.position
        glLoadIdentity()
        yaw_rad = math.radians(self.yaw)

        if mode == 'chase':
            dist = 15.0
            height = 6.0
            cam_x = pos[0] - math.sin(yaw_rad) * dist
            cam_y = pos[1] + height
            cam_z = pos[2] - math.cos(yaw_rad) * dist
            look_x = pos[0] + math.sin(yaw_rad) * 3
            look_y = pos[1] + 1.0
            look_z = pos[2] + math.cos(yaw_rad) * 3
            gluLookAt(cam_x, cam_y, cam_z, look_x, look_y, look_z, 0, 1, 0)
            
        elif mode == 'side':
            side_dist = 20.0
            cam_x = pos[0] - math.sin(yaw_rad + math.pi/2) * side_dist
            cam_y = pos[1] + 8
            cam_z = pos[2] - math.cos(yaw_rad + math.pi/2) * side_dist
            look_x = pos[0]
            look_y = pos[1] + 1.0
            look_z = pos[2]
            gluLookAt(cam_x, cam_y, cam_z, look_x, look_y, look_z, 0, 1, 0)
            
        elif mode == 'top':
            cam_x = pos[0]
            cam_y = pos[1] + 50
            cam_z = pos[2]
            gluLookAt(cam_x, cam_y, cam_z, pos[0], pos[1], pos[2], -math.sin(yaw_rad), 0, -math.cos(yaw_rad))
            
        else:  # hood camera
            cam_x = pos[0] + math.sin(yaw_rad) * 0.5
            cam_y = pos[1] + 1.2
            cam_z = pos[2] + math.cos(yaw_rad) * 0.5
            look_x = pos[0] + math.sin(yaw_rad) * 20
            look_y = pos[1] + 1.0
            look_z = pos[2] + math.cos(yaw_rad) * 20
            gluLookAt(cam_x, cam_y, cam_z, look_x, look_y, look_z, 0, 1, 0)

    def render(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self.update_camera()

        # Draw terrain
        if self.terrain_list and glIsList(self.terrain_list):
            glPushMatrix()
            glTranslatef(0, 0, 0)
            glCallList(self.terrain_list)
            glPopMatrix()
        else:
            print("⚠️  Terrain list not ready or invalid")

        # Draw car shadow (more realistic and visible)
        quad = gluNewQuadric()
        glPushMatrix()
        glTranslatef(self.position[0], 0.01, self.position[2])
        glRotatef(self.yaw, 0, 1, 0)
        glRotatef(90, 1, 0, 0)
        glScalef(2.5, 4.5, 1.0)  # Larger, more realistic shadow
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glDisable(GL_LIGHTING)
        glColor4f(0.0, 0.0, 0.0, 0.6)  # Darker shadow for better visibility
        gluDisk(quad, 0, 1.0, 20, 1)
        glEnable(GL_LIGHTING)
        glDisable(GL_BLEND)
        glPopMatrix()
        
        # Additional ground contact shadow
        glPushMatrix()
        glTranslatef(self.position[0], 0.005, self.position[2])
        glRotatef(self.yaw, 0, 1, 0)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glDisable(GL_LIGHTING)
        glColor4f(0.1, 0.1, 0.1, 0.8)  # Solid dark shadow for ground contact
        glScalef(2.0, 0.1, 3.5)
        self._draw_rounded_box()
        glEnable(GL_LIGHTING)
        glDisable(GL_BLEND)
        glPopMatrix()

        # Draw clouds
        if self.cloud_list and glIsList(self.cloud_list):
            glCallList(self.cloud_list)

        # Draw car
        glPushMatrix()
        glTranslatef(self.position[0], self.position[1], self.position[2])
        glRotatef(self.yaw, 0, 1, 0)
        self.draw_car()
        glPopMatrix()

        # Draw HUD
        speed = abs(self.speed)
        fps_text = f"FPS: {int(self.clock.get_fps())}  Speed: {speed:.1f}  Camera: {self.camera_modes[self.camera_index].upper()}"
        car_text = f"Throttle: {self.throttle_input:.2f}  Steering: {self.steering_angle:.1f}°  Direction: {self.yaw:.1f}°"
        self._draw_hud(fps_text, car_text)

        # Draw joystick visualization
        self._draw_joystick_hud()

    def _draw_hud(self, text1, text2=""):
        """Draw HUD text overlay"""
        surf1 = self.font.render(text1, True, (255, 255, 255))
        data1 = pygame.image.tostring(surf1, "RGBA", True)
        glWindowPos2d(8, self.display[1] - 24)
        glDrawPixels(surf1.get_width(), surf1.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, data1)
        
        if text2:
            surf2 = self.font.render(text2, True, (255, 255, 255))
            data2 = pygame.image.tostring(surf2, "RGBA", True)
            glWindowPos2d(8, self.display[1] - 48)
            glDrawPixels(surf2.get_width(), surf2.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, data2)

    def _draw_joystick_hud(self):
        """Draw joystick visualization overlay"""
        hud_w, hud_h = 160, 160
        margin = 8
        x0 = self.display[0] - hud_w - margin
        y0 = self.display[1] - hud_h - margin

        sm = self.bt_controller.get_smoothed_input() if self.bt_controller else {'x': 512, 'y': 512, 'button': 0}
        raw = getattr(self.bt_controller, 'raw_data', {'x': 512, 'y': 512, 'button': 0})
        connected = getattr(self.bt_controller, 'connected', False)

        surf = pygame.Surface((hud_w, hud_h), flags=SRCALPHA)
        surf.fill((0, 0, 0, 90))

        pygame.draw.rect(surf, (220, 220, 220), (0, 0, hud_w, hud_h), 1)
        label = "BT: ON" if connected else "BT: OFF"
        surf.blit(self.font.render(label, True, (255, 255, 255)), (6, 6))

        cx = hud_w // 2
        cy = hud_h // 2 + 10
        radius = 56
        pygame.draw.circle(surf, (150, 150, 150), (cx, cy), radius, 1)

        rx = int(cx + (raw.get('x', 512) - 512) / 512.0 * radius)
        ry = int(cy + (raw.get('y', 512) - 512) / 512.0 * -radius)
        pygame.draw.circle(surf, (200, 60, 60), (rx, ry), 5)

        sx = int(cx + (sm.get('x', 512) - 512) / 512.0 * radius)
        sy = int(cy + (sm.get('y', 512) - 512) / 512.0 * -radius)
        pygame.draw.circle(surf, (80, 220, 120), (sx, sy), 5)

        surf.blit(self.font.render(f"Raw: {raw.get('x', 512)},{raw.get('y', 512)}", True, (255, 255, 255)), (6, hud_h - 44))
        surf.blit(self.font.render(f"Sm:  {sm.get('x', 512)},{sm.get('y', 512)}", True, (255, 255, 255)), (6, hud_h - 24))

        data = pygame.image.tostring(surf, "RGBA", True)
        glWindowPos2d(x0, y0)
        glDrawPixels(hud_w, hud_h, GL_RGBA, GL_UNSIGNED_BYTE, data)

    def run(self):
        """Main simulation loop"""
        self.build_display_lists()
        
        while self.running:
            self.handle_events()
            self.update_physics()
            self.render()
            
            pygame.display.flip()
            self.clock.tick(self.fps)
        
        # Cleanup
        if self.bt_controller:
            self.bt_controller.stop()
        pygame.quit()


# ---------------------- Entry Point ----------------------
def main():
    try:
        print("🚗 Starting 3D Car Simulator...")
        sim = CarSimulator3D()
        sim.run()
    except Exception as e:
        import traceback
        traceback.print_exc()
        print("Exiting due to error:", e)
        input("Press Enter to exit...")


if __name__ == '__main__':
    main()